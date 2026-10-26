"""CAN 프레임 디코더.

DrumRobot/src/CommandParser.cpp 의 비트 패킹을 그대로 역으로 푼다.

방향(tx/rx) 판별 규칙
---------------------
플로터는 버스를 옆에서 듣기만 하므로, 프레임이 명령인지 피드백인지는
CAN ID 와 프로토콜 규칙으로 추정해야 한다.

  Servo 모드 (TMotorServoCommandParser)
    - 명령(tx)   : 확장 프레임(EFF). can_id = nodeId | (packetId << 8)
    - 피드백(rx) : 확장 프레임(EFF). can_id = nodeId | (STATUS(0x29) << 8), 8바이트
      실장비 로그로 확인했다 (node 5 -> 0x2905). 명령과 피드백이 둘 다 확장 프레임이라
      packet id 로 갈라야 한다. 로봇 C++ 코드는 can_id & 0xFF 로 모터만 찾고 방향을
      따지지 않아서(자기가 보낸 건 자기가 아니까) 이 구분이 원본에는 없다.
      표준 프레임으로 피드백을 주는 펌웨어도 있어서 아래 3) 에 폴백을 남겨둔다.

  MIT 모드 (TMotorMITCodec)
    - 명령(tx)   : 표준 프레임, can_id == nodeId, 8바이트
                   FF FF FF FF FF FF FF FC/FD/FE 는 모드 진입/종료/영점 특수 명령
    - 피드백(rx) : 표준 프레임, data[0] == nodeId, 8바이트
                   (id, pos16, vel12, cur12, 온도, 에러). 6바이트 펌웨어도 받는다.
      실장비 로그(2026-09-29)로 확인한 응답 ID 는 모터마다 달랐다:
        대부분 can_id == nodeId (명령과 같은 ID!), node 3 은 can_id == 마스터 ID(0).
      명령과 피드백이 같은 ID + 같은 8바이트로 오므로 ID 만으로는 못 가른다.
      그래서 data[0] == nodeId 이고 응답 ID 가 (nodeId 또는 마스터 ID) 이면 피드백으로 본다.
      명령의 data[0] 은 목표 위치 상위 바이트라, 이게 nodeId(0~11) 와 같으려면
      목표 위치가 -12.5 rad 근처(범위 끝)여야 해서 실제로는 겹치지 않는다.

  Servo 피드백(표준 프레임 폴백)과 MIT 명령은 둘 다 "표준 프레임 + can_id==nodeId
  + 8바이트" 라서 프레임만 봐서는 구분이 안 된다. 그래서 GUI에서 고른 모터별
  모드로 어느 쪽으로 해석할지 결정한다.

  Maxon (CANopen PDO/SDO) - ID 는 로봇 설정의 pdo id 를 그대로 쓴다
    - 피드백(rx) : rx_pdo_ids[0] (= 0x180 + nodeId). statusword + 실제 위치 + 실제 토크
    - 명령(tx)   : tx_pdo_ids[1] (= 0x300 + nodeId) 은 목표 위치(CSP),
                   tx_pdo_ids[3] (= 0x500 + nodeId) 은 목표 토크(CST) 라 숫자로 푼다.
                   준비 동작은 CSP, 연주는 CST 로 도중에 바뀌므로 둘 다 풀어야
                   명령 그래프가 끊기지 않는다.
                   나머지 PDO 와 SDO(0x600 + nodeId) 는 종류가 많아 raw 만 표시.
"""

import math
import struct

from motor_config import (
    KD_MAX,
    KD_MIN,
    KP_MAX,
    KP_MIN,
    MIT_MASTER_ID,
    MAXON_ENCODER_COUNTS_PER_REV,
    MAXON_RATED_TORQUE_MNM,
    MAXON_SDO_RX_BASE,
    MAXON_SDO_TX_BASE,
    POLE,
    SERVO_STATUS_PACKET_ID,
    gear_ratio,
    P_MAX,
    P_MIN,
    motor_limits,
)

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

# CommandParser.cpp :: enum class CAN_PACKET_ID
SERVO_PACKET_NAMES = {
    0: "SET_DUTY",
    1: "SET_CURRENT",
    2: "SET_DUTY_CURRENT_BRAKE",
    3: "SET_RPM",
    4: "SET_POS",
    5: "SET_ORIGIN_HERE",
    6: "SET_POS_SPD",
    SERVO_STATUS_PACKET_ID: "STATUS",
}
SERVO_SET_RPM = 3
SERVO_SET_POS = 4
SERVO_SET_POS_SPD = 6


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return x_int * span / float((1 << bits) - 1) + x_min


# ---------------------------------------------------------------- Servo mode


def erpm_to_rad_s(erpm, motor):
    """전기적 RPM -> 관절 속도[rad/s].

    controller.cpp 의 정변환을 그대로 뒤집는다:
        erpm = joint_vel * pole * gear_ratio * 60 / (2*pi)
    기어비는 모터마다 다르므로(AK10_9=9, AK70_10=10) 로봇 설정에서 읽는다.
    """
    return erpm * 2.0 * math.pi / (60.0 * POLE * gear_ratio(motor))


def decode_servo_feedback(data, motor):
    """TMotorServoCommandParser::motor_receive"""
    pos_int, spd_int, cur_int = struct.unpack(">hhh", data[0:6])
    temp, error = struct.unpack(">bb", data[6:8])
    # 속도 필드는 ERPM(단위 10)이다. C++ 원본 주석이 "radians per second" 라고
    # 돼 있지만 실제로는 환산 없이 그대로 담긴 값이라 주석이 틀렸다.
    # 그대로 두면 명령(rad/s)과 축이 200배 어긋나 명령선이 0에 눌린다.
    erpm = spd_int * 10.0
    return {
        "pos": pos_int * 0.1 * math.pi / 180.0,
        "vel": erpm_to_rad_s(erpm, motor),
        "erpm": erpm,
        "tor": cur_int * 0.01,  # 전류[A] (토크 상수 미적용)
        "temp": temp,
        "error": error,
    }


def decode_servo_command(packet_id, data, motor):
    """확장 ID 상위 바이트의 packet id 에 따라 해석."""
    name = SERVO_PACKET_NAMES.get(packet_id, "PACKET_%d" % packet_id)
    decoded = {"packet": name}

    if packet_id == SERVO_SET_POS and len(data) >= 4:
        # setPositionCANFrame: pos_deg * 10000 을 int32 big-endian 으로
        pos_int = struct.unpack(">i", data[0:4])[0]
        decoded["pos"] = (pos_int / 10000.0) * math.pi / 180.0
    elif packet_id == SERVO_SET_RPM and len(data) >= 4:
        # setVelocityCANFrame: ERPM 을 int32 big-endian 으로
        erpm = struct.unpack(">i", data[0:4])[0]
        # 이 값은 순수 속도 지령이 아니다. controller.cpp 가 위치 오차 보정항
        # (control_gain * err) 을 더해서 보내므로 피드백과 정확히 겹치지 않는다.
        decoded["vel"] = erpm_to_rad_s(erpm, motor)
        decoded["erpm"] = erpm
    elif packet_id == SERVO_SET_POS_SPD and len(data) >= 8:
        pos_int = struct.unpack(">i", data[0:4])[0]
        spd, rpa = struct.unpack(">hh", data[4:8])
        decoded["pos"] = (pos_int / 10000.0) * math.pi / 180.0
        decoded["vel"] = erpm_to_rad_s(spd * 10, motor)
        decoded["erpm"] = spd * 10
        decoded["rpa"] = rpa * 10

    return decoded


# ------------------------------------------------------------------ MIT mode


MIT_SPECIAL_COMMANDS = {0xFC: "ENTER_MODE", 0xFD: "EXIT_MODE", 0xFE: "SET_ZERO"}


def mit_special_command(data):
    """FF*7 + FC/FD/FE 면 이름, 아니면 None. 값이 아니라 제어 명령이라 디코딩하면 안 된다."""
    if len(data) == 8 and data[:7] == b"\xff" * 7:
        return MIT_SPECIAL_COMMANDS.get(data[7], "SPECIAL_%02X" % data[7])
    return None


def decode_mit_command(data, motor):
    """TMotorMITCodec::encodeCommand 의 역변환."""
    lim = motor_limits(motor)
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | (data[3] >> 4)
    kp_int = ((data[3] & 0x0F) << 8) | data[4]
    kd_int = (data[5] << 4) | (data[6] >> 4)
    t_int = ((data[6] & 0x0F) << 8) | data[7]
    return {
        "pos": uint_to_float(p_int, P_MIN, P_MAX, 16),
        "vel": uint_to_float(v_int, -lim["v"], lim["v"], 12),
        "kp": uint_to_float(kp_int, KP_MIN, KP_MAX, 12),
        "kd": uint_to_float(kd_int, KD_MIN, KD_MAX, 12),
        "tor": uint_to_float(t_int, -lim["t"], lim["t"], 12),
    }


def decode_mit_feedback(data, motor):
    """TMotorMITCodec::decodeFeedback 의 역변환."""
    lim = motor_limits(motor)
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    i_int = ((data[4] & 0x0F) << 8) | data[5]

    i_max = lim["iMax"] if lim.get("iMax") else lim["t"]
    current = uint_to_float(i_int, -i_max, i_max, 12)
    kt = lim["kt"]

    decoded = {
        "pos": uint_to_float(p_int, P_MIN, P_MAX, 16),
        "vel": uint_to_float(v_int, -lim["v"], lim["v"], 12),
        # Kt 를 아는 기종이면 N·m, 모르면 전류[A] 그대로
        "tor": current * kt if kt else current,
        "nodeIdByte": data[0],
    }
    # 8바이트 펌웨어: data[6] 온도(+40 오프셋, CubeMars 매뉴얼), data[7] 에러 코드.
    # 6바이트 펌웨어도 dlc 8 에 0 으로 채워 보내는 경우가 있어서(실장비 node 3)
    # 온도 원시값 0(= -40°C) 은 "온도 필드 없음" 으로 본다.
    if len(data) >= 8 and data[6] != 0:
        decoded["temp"] = data[6] - 40
        decoded["error"] = data[7]
    return decoded


# --------------------------------------------------------------------- Maxon


def decode_maxon_feedback(data):
    """MaxonMotorCodec::decodeFeedback (TPDO1: statusword + 위치 + 토크)"""
    status_bit = data[1]
    pos_counts = struct.unpack("<i", data[2:6])[0]
    torque_raw = struct.unpack("<h", data[6:8])[0]
    pos_deg = (pos_counts / MAXON_ENCODER_COUNTS_PER_REV) * 360.0
    return {
        "pos": pos_deg * math.pi / 180.0,
        "tor": (torque_raw / 1000.0) * MAXON_RATED_TORQUE_MNM,
        "statusBit": status_bit,
    }


def decode_maxon_position_command(data):
    """MaxonMotorCodec::encodePosition 의 역변환 (RPDO2: 목표 위치, little-endian)"""
    pos_counts = struct.unpack("<i", data[0:4])[0]
    pos_deg = (pos_counts / MAXON_ENCODER_COUNTS_PER_REV) * 360.0
    return {"packet": "TARGET_POS", "pos": pos_deg * math.pi / 180.0}


def decode_maxon_torque_command(data):
    """MaxonMotorCodec::encodeTorque 의 역변환 (RPDO4: 목표 토크, dlc 2).

    로봇 쪽은 mN.m 을 정격토크의 1/1000 단위로 바꿔서 int16 으로 싣는다.
    피드백(decode_maxon_feedback) 과 같은 환산을 써야 명령/피드백이 같은 축에 겹친다.
    """
    torque_raw = struct.unpack("<h", data[0:2])[0]
    return {
        "packet": "TARGET_TOR",
        "tor": (torque_raw / 1000.0) * MAXON_RATED_TORQUE_MNM,
    }


# ---------------------------------------------------------------- dispatcher


def classify(can_id_raw, data, motors_by_node_map, mode_of):
    """프레임 하나를 (motor, dir, decoded) 로 변환. 모르는 프레임이면 None.

    mode_of(motor) 는 "servo" / "mit" / "maxon" 을 돌려주는 콜백.
    """
    if can_id_raw & (CAN_ERR_FLAG | CAN_RTR_FLAG):
        return None

    is_eff = bool(can_id_raw & CAN_EFF_FLAG)
    can_id = can_id_raw & (CAN_EFF_MASK if is_eff else CAN_SFF_MASK)
    dlc = len(data)

    # 1) Maxon (CANopen PDO / SDO)
    if not is_eff:
        for node, motor in motors_by_node_map.items():
            if motor["type"] != "Maxon":
                continue

            # 피드백: TPDO1 (statusword + 실제 위치 + 실제 토크)
            if can_id == motor.get("rxPdo") and dlc >= 8:
                return motor, "rx", decode_maxon_feedback(data)

            # 명령: 목표 위치 PDO 는 숫자로 풀고, 나머지(control word, SDO)는 raw 만
            if can_id == motor.get("positionPdo") and dlc >= 4:
                return motor, "tx", decode_maxon_position_command(data)
            if can_id == motor.get("torquePdo") and dlc >= 2:
                return motor, "tx", decode_maxon_torque_command(data)
            if can_id in motor.get("txPdos", []):
                return motor, "tx", {"packet": "PDO"}
            if can_id == MAXON_SDO_TX_BASE + node:
                return motor, "tx", {"packet": "SDO"}
            if can_id == MAXON_SDO_RX_BASE + node:
                return motor, "rx", {"packet": "SDO"}

    # 2) Servo: 확장 프레임, 하위 바이트가 nodeId, 상위 바이트가 packet id.
    #    STATUS(0x29) 만 피드백이고 나머지 packet id 는 전부 명령이다.
    if is_eff:
        motor = motors_by_node_map.get(can_id & 0xFF)
        if motor and motor["type"] == "TMotor":
            packet_id = (can_id >> 8) & 0xFF
            if packet_id == SERVO_STATUS_PACKET_ID and dlc >= 8:
                return motor, "rx", decode_servo_feedback(data, motor)
            return motor, "tx", decode_servo_command(packet_id, data, motor)
        return None

    # NMT 는 ID 0 이라 마스터 ID/node 0 과 겹친다. 모터 값이 아니므로 버린다.
    if _looks_like_nmt(can_id, data):
        return None

    # 3) MIT 피드백: data[0] 이 nodeId, 응답 ID 는 nodeId 또는 마스터 ID.
    #    명령과 ID 가 같을 수 있으므로 명령(4)보다 먼저 본다.
    special = mit_special_command(data)
    if dlc >= 6 and special is None:
        motor = motors_by_node_map.get(data[0])
        if (motor and motor["type"] == "TMotor" and mode_of(motor) == "mit"
                and can_id in (motor["nodeId"], MIT_MASTER_ID)):
            return motor, "rx", decode_mit_feedback(data, motor)

    # 4) 표준 프레임: 모터별로 선택된 모드에 따라 Servo 피드백 / MIT 명령으로 해석.
    #    실장비 서보 피드백은 2) 로 들어오므로 여기 servo 경로는 폴백이다.
    #    표준 ID 는 11비트라 & 0xFF 로 자르면 0x200 같은 남의 프레임이 node 0 에 붙는다.
    motor = motors_by_node_map.get(can_id)
    if motor and motor["type"] == "TMotor":
        mode = mode_of(motor)
        if mode == "servo" and dlc >= 8:
            return motor, "rx", decode_servo_feedback(data, motor)
        if mode == "mit" and dlc >= 8:
            if special:
                return motor, "tx", {"packet": special}
            return motor, "tx", decode_mit_command(data, motor)

    return None


def _looks_like_nmt(can_id, data):
    """CANopen NMT(ID 0x000, [명령, 노드, 0...]) 를 MIT 피드백으로 오인하지 않게 거른다.

    마스터 ID 가 0 이면 NMT 와 ID 가 겹친다. 실장비 로그에 Maxon 시작용
    01 07 00 00 00 00 00 00 이 있었는데, 그대로 두면 node 1 피드백(-12.4 rad)으로 찍힌다.
    """
    return (can_id == 0 and len(data) >= 2
            and data[0] in (0x01, 0x02, 0x80, 0x81, 0x82) and not any(data[2:]))
