"""모터 구성과 프로토콜 상수.

모터 목록(이름·노드 ID·종류)은 **로봇 코드(Phil)의 config/motors.json 을 읽기 전용으로
참조**한다. 로봇 쪽에서 ID 를 바꾸면 플로터가 자동으로 따라간다.
로봇 설정을 못 찾으면 config/motors.json 의 fallbackMotors 로 동작한다.

로봇 코드에는 어떤 방식으로도 쓰지 않는다. 이 도구는 CAN 을 보기만 한다.

경로 결정 순서:
  1) 환경변수 PLOTTER_ROBOT_CONFIG
  2) config/motors.json 의 robotConfigPath
  3) config/motors.json 의 robotConfigSearch 목록
"""

import json
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "motors.json"

with open(str(CONFIG_PATH), encoding="utf-8") as _f:
    CONFIG = json.load(_f)

DEFAULT_MODE = CONFIG.get("defaultMode", "servo")
TMOTOR_LIMITS = CONFIG["motorTypes"]
DEFAULT_MOTOR_TYPE = next(iter(TMOTOR_LIMITS))

_tmotor = CONFIG["tmotor"]
P_MIN = _tmotor["pMin"]
P_MAX = _tmotor["pMax"]
KP_MIN = _tmotor["kpMin"]
KP_MAX = _tmotor["kpMax"]
KD_MIN = _tmotor["kdMin"]
KD_MAX = _tmotor["kdMax"]
POLE = _tmotor["pole"]
# 서보 모드 상태(피드백) 프레임의 packet id. 확장 ID 상위 바이트로 들어온다 (0x29 = 41).
SERVO_STATUS_PACKET_ID = _tmotor.get("servoStatusPacketId", 41)
# MIT 응답이 자기 nodeId 대신 이 ID 로 오는 모터가 있다 (실장비 로그: node 3 -> 0x000)
MIT_MASTER_ID = _tmotor.get("mitMasterId", 0)

_maxon = CONFIG["maxon"]
MAXON_FEEDBACK_PDO_BASE = _maxon["feedbackPdoBase"]   # 0x180 + node : 모터 -> 제어 프로세스
MAXON_CONTROL_PDO_BASE = _maxon["controlPdoBase"]     # 0x200 + node : control word
MAXON_POSITION_PDO_BASE = _maxon["positionPdoBase"]   # 0x300 + node : 목표 위치
MAXON_TORQUE_PDO_BASE = _maxon["torquePdoBase"]       # 0x500 + node : 목표 토크 (CST)
MAXON_SDO_TX_BASE = _maxon["sdoTxBase"]               # 0x600 + node : SDO 요청
MAXON_SDO_RX_BASE = _maxon["sdoRxBase"]               # 0x580 + node : SDO 응답
MAXON_RATED_TORQUE_MNM = _maxon["ratedTorqueMNm"]
MAXON_ENCODER_COUNTS_PER_REV = _maxon["encoderCountsPerRev"]


def _as_int(value):
    if isinstance(value, str):
        return int(value, 16 if value.lower().startswith("0x") else 10)
    return int(value)


def _prettify(name):
    return name.replace("_", " ").title()


def _find_robot_config():
    candidates = []
    env_path = os.environ.get("PLOTTER_ROBOT_CONFIG")
    if env_path:
        candidates.append(env_path)
    if CONFIG.get("robotConfigPath"):
        candidates.append(CONFIG["robotConfigPath"])
    candidates.extend(CONFIG.get("robotConfigSearch", []))

    for candidate in candidates:
        path = Path(os.path.expanduser(candidate))
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        if path.is_file():
            return path.resolve()
    return None


def _convert_robot_motors(robot_motors):
    """로봇 설정 스키마 -> 플로터 내부 형식. CAN 을 쓰지 않는 모터는 건너뛴다."""
    converted = []
    for motor in robot_motors:
        kind = motor.get("type")
        if kind == "TMotor":
            plotter_type = "TMotor"
        elif kind in ("MaxonMotor", "Maxon"):
            plotter_type = "Maxon"
        else:
            continue  # Dynamixel 등은 CAN 버스에 안 나온다

        node_id = _as_int(motor["node_id"])
        entry = {
            "id": motor.get("name", "motor_%d" % node_id),
            "label": _prettify(motor.get("name", "motor %d" % node_id)),
            "nodeId": node_id,
            "type": plotter_type,
            # ERPM <-> 관절 속도 환산에 필요하다 (controller.cpp 가 기어비를 곱해서 보낸다)
            "gearRatio": float(motor.get("gear_ratio", 1.0)),
        }

        if plotter_type == "TMotor":
            entry["motorType"] = motor.get("model", DEFAULT_MOTOR_TYPE)
        else:
            # PDO ID 는 로봇 설정에 있으면 그대로, 없으면 표준 오프셋으로 계산
            entry["rxPdo"] = _as_int(motor.get("rx_pdo_ids[0]", MAXON_FEEDBACK_PDO_BASE + node_id))
            tx_pdos = []
            for index in range(4):
                key = "tx_pdo_ids[%d]" % index
                if key in motor:
                    tx_pdos.append(_as_int(motor[key]))
            entry["txPdos"] = tx_pdos or [MAXON_CONTROL_PDO_BASE + node_id,
                                          MAXON_POSITION_PDO_BASE + node_id]
            entry["positionPdo"] = tx_pdos[1] if len(tx_pdos) > 1 else MAXON_POSITION_PDO_BASE + node_id
            # 연주 중에는 CSP(목표 위치) 가 아니라 CST(목표 토크) 로 명령이 나간다.
            # controller.cpp 의 maxon_motor_send_task() 가 tx_pdo_ids[3] 로 encodeTorque() 를 쏜다.
            entry["torquePdo"] = tx_pdos[3] if len(tx_pdos) > 3 else MAXON_TORQUE_PDO_BASE + node_id

        converted.append(entry)
    return converted


ROBOT_CONFIG_PATH = _find_robot_config()

if ROBOT_CONFIG_PATH:
    with open(str(ROBOT_CONFIG_PATH), encoding="utf-8") as _f:
        MOTORS = _convert_robot_motors(json.load(_f)["motors"])
    CONFIG_SOURCE = str(ROBOT_CONFIG_PATH)
else:
    MOTORS = CONFIG["fallbackMotors"]
    for _m in MOTORS:
        _m.setdefault("gearRatio", 1.0)
        if _m["type"] == "Maxon":
            _m.setdefault("rxPdo", MAXON_FEEDBACK_PDO_BASE + _m["nodeId"])
            _m.setdefault("positionPdo", MAXON_POSITION_PDO_BASE + _m["nodeId"])
            _m.setdefault("torquePdo", MAXON_TORQUE_PDO_BASE + _m["nodeId"])
            _m.setdefault("txPdos", [MAXON_CONTROL_PDO_BASE + _m["nodeId"], _m["positionPdo"],
                                     _m["torquePdo"]])
    CONFIG_SOURCE = "fallback (로봇 설정을 찾지 못함)"


def motors_by_node():
    return {m["nodeId"]: m for m in MOTORS}


def gear_ratio(motor):
    return float(motor.get("gearRatio", 1.0)) or 1.0


def motor_limits(motor):
    return TMOTOR_LIMITS.get(motor.get("motorType"), TMOTOR_LIMITS[DEFAULT_MOTOR_TYPE])
