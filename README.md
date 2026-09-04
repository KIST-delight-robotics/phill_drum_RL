# phill_drum_RL

드럼 로봇 실시간 제어 서버. **연주 구간의 팔 9관절을 강화학습 정책이 폐루프로 구동**합니다.
머리·발·전이 궤적은 기존 개루프 경로를 그대로 씁니다.

> 원본 저장소 [`Phil-drum-robot`](https://github.com/KIST-delight-robotics/Phil-drum-robot) 에서
> RL 정책 이식 작업을 분리한 저장소입니다. 정책을 켜지 못하는 상황에서는
> 모든 경로가 기존 개루프로 안전 강등되므로, 원본과 같은 방식으로도 동작합니다.

> 개발 중인 드럼 연주 로봇 제어 시스템.

CAN 통신 기반으로 TMotor / Maxon / Dynamixel 모터를 제어하며, LLM(TCP)을 통해 명령을 수신합니다. 이름은 드러머 Phil Collins에서 따왔습니다.

---

## 시스템 요구사항

### 운영체제 및 권한

- **OS**: Ubuntu 22.04 LTS
- **권한**: `SCHED_FIFO` 우선순위 설정 및 CAN 포트 제어를 위해 `sudo` 실행 필요

### 빌드 환경

- **컴파일러**: g++ (C++17 지원)
- **빌드 도구**: GNU Make
- **컴파일 옵션**: `-Wall -O2 -g -std=c++17 -fPIC` (Makefile 기본값)

### 의존성

**저장소에 포함**

- **Dynamixel SDK** — `drumrobot_server/lib/dynamixel_sdk/`
- **nlohmann/json** — `drumrobot_server/lib/nlohmann/json.hpp`
- **miniaudio** — `drumrobot_server/lib/miniaudio/miniaudio.h`

**따로 받아야 하는 것 — ONNX Runtime 1.23.2**

정책 추론에 씁니다. 용량이 커서 저장소에 넣지 않고 스크립트로 받습니다
(sha256 고정, 최초 1회):

```bash
bash drumrobot_server/lib/onnxruntime/fetch.sh
```

받지 않아도 **빌드는 실패하지만**, 받은 뒤에는 정책 없이도(=개루프로도) 그대로 동작합니다.

### 하드웨어

- USB-CAN 어댑터 (`can0`, `can1` 등)
- Dynamixel U2D2 (`/dev/ttyUSB0`)
- USB 허브 전원 제어용 `uhubctl` — CAN 포트 리셋에 사용

---

## 새 컴퓨터에서 처음 돌리기

`git pull` 이후 필요한 것은 **ONNX Runtime 내려받기 하나**입니다.
정책 모델(`policy.onnx`)과 상수(`obs_constants.json`)는 저장소에 포함돼 있어
따로 옮기지 않습니다.

### 1. 저장소

```bash
git remote -v                       # rl 원격이 있는지 확인
git fetch rl
git checkout main
git pull rl main
```

`policy.onnx` (410KB) 와 `obs_constants.json` 이 함께 내려옵니다.
학습 쪽에서 정책을 다시 export 했다면 그 두 파일만 갱신해 커밋하면 됩니다.

### 2. ONNX Runtime (최초 1회, 네트워크 필요)

```bash
bash drumrobot_server/lib/onnxruntime/fetch.sh
```

v1.23.2 를 받아 sha256 을 검증합니다. Makefile 이 `$ORIGIN` 상대 rpath 로
링크하므로 **시스템 설치는 필요 없습니다.** 이미 있으면 건너뜁니다.

> `onnxruntime-linux-**x64**` 를 받습니다. x86-64 리눅스여야 합니다.

### 3. 빌드

```bash
make -C drumrobot_server
```

### 4. 로봇 없이 확인 — 여기까지는 모터 없이 됩니다

```bash
make -C drumrobot_server ort-check       # 모델 로드 + 추론 지연
make -C drumrobot_server golden-check    # 관측 조립을 sim 덤프와 대조
make -C drumrobot_server score-check     # 악보 파싱 · 곡 밀도
```

| 도구 | 통과 기준 |
|---|---|
`ort-check` | 세션 생성 성공, p99 추론 시간이 예산(15,000µs) 대비 충분히 작음 |
`golden-check` | `joint_pos` · `joint_vel` · `drum_pos` · `arm_role` · `per_arm_pos` · `per_arm_time` 오차 0 |
| | `tip_pos` · `hit_armed` 불일치는 **스틱 길이 미확정(373 vs 385mm)** 에서 오는 알려진 항목 |

**`ort-check` 가 실패하면 2번을 다시 하십시오.** 그 외 단계로 넘어가도 소용없습니다.

### 5. 이 머신을 `can_ports.json` 에 등록

USB-CAN 어댑터를 강제 재인식시키는 데 쓰는 항목입니다.
**등록하지 않아도 동작합니다** — 리셋만 건너뜁니다.

```bash
hostname                            # 이 값이 키가 된다
```

```json
{
  "machines": {
    "shy-desktop":            { "hub": "1-4",   "ports": [1, 2, 3, 4] },
    "shy-MINIPC-VC66-C2":     { "hub": "1-6.1", "ports": [1] },
    "shy":                    { "hub": "",      "ports": [] }
  }
}
```

- `hub` — **그 컴퓨터의 USB 토폴로지 주소.** `uhubctl -l` 로 확인합니다.
  다른 머신 값을 복사하면 없는 허브를 껐다 켜려다 실패합니다.
- `ports` — 어댑터가 꽂힌 허브 포트 번호
- 둘 중 하나가 비면 `No CAN reset needed for <host>` 를 찍고 넘어갑니다

```bash
command -v uhubctl || sudo apt install uhubctl
```

### 6. CAN 인터페이스

**인터페이스를 올리는 것은 코드가 합니다.** `ip link show | grep can` 으로
존재하는 `canN` 을 모두 찾아 1 Mbps 로 올립니다 (`activateCanPort`).
`make run` 이 sudo 로 돌기 때문에 미리 올려둘 필요가 없습니다.

**어댑터 개수는 로봇의 CAN 버스 수와 같아야 합니다.**
모터가 어느 버스에 있는지는 `motors.json` 에 적혀 있지 않고,
`set_motors_socket()` 이 **모든 소켓에 물어보고 응답한 곳으로 배정**합니다.
버스가 4개인데 어댑터가 1개면 그 버스의 모터만 잡힙니다.

### 7. 첫 실행

```bash
make run
```

시동 로그에서 **`Connected` 아홉 줄** 을 세십시오:

```
[Robot] --------------> CAN NODE ID 17 Connected. Joint 0: waist
[Robot] --------------> CAN NODE ID 1  Connected. Joint 1: right_shoulder_1
...
[Robot] --------------> CAN NODE ID 8  Connected. Joint 8: left_wrist
[PolicyRunner] 준비 완료 — 주기 15ms (66.6667Hz), stride 3
```

**`Joint 0` ~ `Joint 8` 이 전부 있어야 정책이 팔을 인수합니다.**
하나라도 빠지면 `팔 모터 N 결번 — 개루프로 연주합니다` 를 찍고
**연주는 계속되지만 정책은 조용히 꺼집니다.**

첫 회차는 `PLAY` 없이 `START` → `READY` → `QUIT` 으로
토크 인가·유지·해제만 확인하십시오.

---

## CAN 통신이 안 될 때

모터가 전부 `Not Connected` 로 나오면 개별 설정이 아니라 버스 문제입니다.
`can-utils` 없이 확인할 수 있습니다.

### 상태 먼저

```bash
ip -d link show can0 | grep -E "state|can state|bitrate"
```

```
can state ERROR-ACTIVE  (berr-counter tx 0 rx 0)      정상
can state ERROR-PASSIVE (berr-counter tx 128 rx 0)    아래 표 참조
state DOWN                                            인터페이스가 내려가 있음
```

인터페이스가 내려가 있으면:

```bash
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

### 판정

| `tx` 카운터 | `rx` | 뜻 | 확인할 것 |
|---|---|---|---|
0 | 0 | 송신 자체가 안 나감 | 어댑터 · 드라이버 · 인터페이스 DOWN |
**증가** | **0** | **보내는데 아무도 안 들음** | 로봇 전원 · CAN 선 · `CAN_H`/`CAN_L` 반전 · 종단저항 120Ω · 선이 그 버스에 닿는지 |
0 | 증가 | 정상 수신 중 | 노드 id · 비트레이트 |

CAN 은 프레임이 성립하려면 **최소 두 노드**가 필요합니다. 보낸 쪽이 ACK 비트를
읽지 못하면 재시도하며 `tx` 가 오르고, 128 에서 `ERROR-PASSIVE` 가 됩니다.
**`tx` 가 오르고 `rx` 가 0 이면 버스에 자기 혼자 있는 것입니다.**

### 노드별로 찔러보기

MIT 모드 모터는 스스로 프레임을 뿌리지 않으므로 수동 청취로는 판단할 수 없습니다.
제어 모드 진입 프레임(`FF FF FF FF FF FF FF FC`)을 보내고 응답을 봅니다.

```bash
# 파이썬 raw CAN 소켓. 별도 설치 불필요
python3 - <<'EOF'
import socket, struct, sys
FMT = "=IB3x8s"
NODES = {0x11:"waist", 0x01:"r_sh1", 0x02:"l_sh1", 0x03:"r_sh2",
         0x04:"r_elb", 0x05:"l_sh2", 0x06:"l_elb"}
s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(("can0",)); s.settimeout(0.15)
try:
    while True: s.recv(struct.calcsize(FMT))     # 묵은 프레임 비우기
except socket.timeout: pass
for nid, name in NODES.items():
    s.send(struct.pack(FMT, nid, 8, bytes([0xFF]*7 + [0xFC])))
    try:
        cid, dlc, d = struct.unpack(FMT, s.recv(struct.calcsize(FMT)))
        hit = " <- data[0] 일치" if dlc and d[0] == nid else ""
        print(f"0x{nid:02X} {name:6s} 응답 can_id 0x{cid & 0x1FFFFFFF:03X} "
              f"{' '.join(f'{b:02X}' for b in d[:dlc])}{hit}")
    except socket.timeout:
        print(f"0x{nid:02X} {name:6s} 무응답")
EOF
```

- **`data[0] 일치`** 가 보이면 MIT 모드까지 확인된 것입니다.
  MIT 는 `can_id` 가 0 으로 오고 식별자가 `data[0]` 에 들어옵니다
  (서보 모드는 `can_id` 하위 바이트).
- 일부만 응답하면 그 노드들만 살아 있는 것이니 CubeMars 설정을 보십시오.
- 전부 무응답이면 위 판정 표로 돌아가십시오.

### 비트레이트 확인

코드는 **1 Mbps 고정**입니다 (`activateCanPort`). CubeMars 에서 MIT 모드를
설정할 때 baud rate 를 함께 바꿨다면 여기서 걸립니다.

```bash
for BR in 1000000 500000 250000 125000; do
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate $BR restart-ms 100
    echo "--- $BR ---"
    # 위 파이썬 조각을 여기서 실행
done
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100   # 복원
```

> `sudo` 자격이 만료되면 `down` 은 되고 `up` 은 실패해 인터페이스가
> 내려간 채로 남을 수 있습니다. 스크립트로 묶어 `sudo` 로 한 번에 돌리십시오.

---

## 운용 절차 (전원 ON ~ 연주)

이 로봇은 안전을 위해 **고정 키(locking pin)** 로 초기 위치를 잡은 뒤 단계적으로 활성화합니다. 절차를 반드시 지켜야 합니다.

1. **고정 키를 꽂은 상태로** 로봇 전원을 켭니다.
2. 코드를 실행하면 모터 통신을 확인하고 대기합니다. 이 시점에는 **모터 토크가 걸려 있지 않습니다** (`Standby`).
3. `START` 입력 → 모터 토크 ON, home 자세 유지 (`Init`). 콘솔에 고정 키 제거 안내가 출력됩니다.
4. **고정 키를 모두 제거**합니다. (토크가 걸려 자세를 붙들고 있는 상태에서 제거)
5. `READY` 입력 → 동작 허용 상태로 전환 (`Idle`). 이후 LOOK / MOVE / POSE / GESTURE / HIT / PLAY 명령을 사용할 수 있습니다.

> 고정 키를 제거하기 전(`Init` 상태)에는 팔을 움직이는 동작 명령이 모두 거부됩니다. 반드시 키 제거 후 `Idle`로 전환하세요.

---

## 시스템 구조

### 데이터 흐름

```
입력 (TCP)
        ↓
   CommandQueue        (문자열 명령)
        ↓
   CommandParser       (명령 파싱 → ParsedCommand)
        ↓
   BehaviorPlanner     (ParsedCommand → MotionPrimitive 시퀀스)
        ↓
   MotionQueue         (MotionPrimitive)
        ↓
   TrajectoryGenerator (궤적 생성 → ControlSetPoint 시퀀스)
        ↓
   ControlQueue        (5ms 주기 ControlSetPoint)
        ↓
   Controller          (CAN 송수신, 1ms / 5ms 주기)
        ↓
   CAN Bus → 모터
```

### 정책 폐루프 (연주 중 팔 0~8)

```
   Controller ──(조건변수, 3틱=15ms)──▶ PolicyRunner
        ▲                                    │
        │                              ONNX 추론 (~20 µs)
        │                                    ▼
        │                              PolicyTarget      (seqlock 슬롯 · 최신값)
        │                                    │
        └────────── merge_policy_target ◀────┘
                    (팔 0~8 목표 덮어쓰기)

   CAN Bus → recv_thread ──▶ JointSnapshot (seqlock · 9관절 시간정합) ──▶ PolicyRunner
```

정책 출력은 **큐가 아니라 슬롯**에 씁니다. `ControlQueue` 에 넣으면 planner 가 미리 채운
약 100ms 버퍼 뒤에 줄을 서 폐루프가 성립하지 않기 때문입니다. 값을 흘려도 되고,
**낡은 값만 막으면** 되므로 슬롯에 `tick` 을 함께 실어 워치독이 검사합니다.

정책 주기는 자체 타이머가 아니라 **`send_loop` 의 틱 3개**로 정의됩니다. 그래서 두 개의
독립된 시계가 생기지 않고, send 가 지터를 겪어도 정렬이 유지됩니다.

### 핵심 컴포넌트

| 컴포넌트 | 책임 |
|---|---|
| `TcpServer` | 사용자 입력(TCP)을 받아 `CommandQueue`에 푸시 |
| `CommandParser` | 문자열 명령을 파싱해 `ParsedCommand`(Opcode + args)로 변환 |
| `BehaviorPlanner` | `ParsedCommand`를 받아 `MotionPrimitive` 시퀀스 생성. 로봇 상태(`RobotState`) 전이 관리 |
| `TrajectoryGenerator` | `MotionPrimitive`를 받아 5ms 단위 `ControlSetPoint` 궤적 생성 (관절/작업공간/연주) |
| `PlayMotionGenerator` | 연주(DRUM) 모션 합성. Base / State / Pedal / Head 4개 서브 제너레이터를 IK로 통합 |
| `MotionPlanner` | 위 컴포넌트를 orchestrate하는 스레드 루프. `CommandQueue` 소비 및 `ControlQueue` 잔량 관리 |
| `KinematicsSolver` | 손 끝 좌표 → 9개 관절각 (역기구학), 9개 관절각 → 손 끝 좌표 (순기구학) |
| `Controller` | `ControlQueue`에서 꺼내 CAN 프레임 송신 (1ms Maxon 보간 + 5ms 동시), 모터 상태 수신 및 안전 검사 |
| `Robot` | 모터 객체들의 컨테이너. 초기화, socket 할당, Maxon/Dynamixel 설정 |
| `CanInterface` | CAN 포트 활성화, 소켓 생성, 프레임 read/write, Non-block 설정 |
| `MotorCodec` | TMotor / MaxonMotor 종류별 CAN 프레임 구성·해석. 명령 프레임 빌드는 `encode*`, 수신 프레임 파싱은 `decodeFeedback` |
| `Logger` | 모터/궤적/명령 상태를 타임스탬프 기반 CSV 파일로 저장 |

### 연주 모션 서브 제너레이터 (`PlayMotionGenerator`)

| 제너레이터 | 담당 관절 | 역할 |
|---|---|---|
| `BaseMotionGenerator` | 팔 (작업공간 좌표 → IK) | 악기 위치로 손 끝 이동 (Bézier 경로) |
| `StateMotionGenerator` | 팔꿈치 / 손목 보정 | 타격 강도에 따른 들어올림·내려침 디테일 |
| `PedalMotionGenerator` | 페달 (9, 10) | 베이스 드럼, 하이햇 open/closed |
| `HeadMotionGenerator` | 머리 (11, 12) | 악기 응시(yaw) + 박자 끄덕임(pitch) |

### 스레드 구조

| 스레드 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `send_thread` | 40 | 1ms (Maxon 보간) / 5ms (TMotor + Maxon + Dynamixel) | CAN 프레임 송신 |
| `recv_thread` | 30 | 100µs | CAN 프레임 수신 및 모터 상태 갱신, 안전 검사 |
| `policy_thread` | 25 | 3틱 = 15ms (66.7Hz) | obs 조립 → ONNX 추론 → 팔 목표 발행 |
| `motion_planning_thread` | 20 | 5ms 폴링 | CommandQueue → MotionQueue → ControlQueue 변환 |
| `tcp_server_thread` | 10 | 블로킹 | 사용자 입력(TCP) 수신 |

우선순위는 `SCHED_FIFO` 정책 기준 (값이 클수록 높음).
`policy_thread` 는 `recv`(30)보다 낮습니다 — 정책이 폭주해도 모터 상태 갱신이 굶으면
안 되기 때문입니다. `motion_planning`(20)보다는 높습니다 — 궤적 생성은 무겁고
데드라인이 없습니다. 정책을 켜지 못하면 이 스레드는 **생성되지 않습니다**.

---

## 파일 구조

```
Phil/
├── Makefile
├── drumrobot_client/
│   └── main.py                             # TCP 클라이언트 (LLM 모드용)
└── drumrobot_server/
    ├── Makefile
    ├── bin/                                # 실행 파일 (빌드 시 생성)
    ├── obj/                                # 오브젝트 파일 (빌드 시 생성)
    ├── log/                                # Logger 출력 디렉토리 (런타임 생성)
    ├── config/
    │   ├── motors.json                     # 모터 설정 (ID, 관절 범위, PDO ID 등)
    │   ├── robot_poses.json                # 사전 정의 포즈 (init / home / ready / shutdown)
    │   ├── kinematics.json                 # 관절 한계, 링크 길이
    │   ├── drum_coordinate.json            # 악기별 손 끝 좌표 및 손목 각도
    │   └── can_ports.json                  # 머신별 USB 허브/포트 매핑 (CAN 리셋용)
    ├── data/
    │   ├── midi/                           # MIDI 원본
    │   ├── scores/                         # 연주용 악보 (.txt)
    │   ├── audio/                          # 음악 파일 (.wav)
    │   └── policy/                         # ★ 학습 산출물
    │       ├── policy.onnx                 #   정책 그래프 (정규화 포함)
    │       ├── obs_constants.json          #   주기·게인·판정상수·관절순서
    │       └── golden.json                 #   검증 픽스처 (미추적, dump_golden.py 로 생성)
    ├── tools/                              # ★ 검증 도구 (main.out 에 미포함)
    │   ├── golden_check.cpp                #   sim vs C++ 요소별 대조
    │   ├── ort_check.cpp                   #   ORT 링크·그래프 규약·추론시간
    │   ├── obs_check.cpp                   #   obs 조립 단위검사
    │   └── policy_score_check.cpp          #   악보 파싱·양자화·밀도
    ├── lib/
    │   ├── dynamixel_sdk/
    │   ├── miniaudio/miniaudio.h
    │   ├── onnxruntime/fetch.sh            # ★ 라이브러리 본체는 미추적
    │   └── nlohmann/json.hpp
    ├── include/
    │   ├── common/                         # app_context, command/control/motion_queue, robot_config
    │   ├── hardware/                       # can_interface, motor, motor_codec, robot
    │   ├── kinematics/                     # kinematics_solver
    │   ├── tcp/                            # tcp_server, command_parser
    │   ├── realtime/                       # controller
    │   ├── policy/                         # ★ policy_runner, obs_builder, policy_score,
    │   │                                   #   midi_score, policy_config, policy_target
    │   ├── trajectory/                     # behavior_planner, motion_planner,
    │   │                                   #   trajectory_generator, play/base/state/pedal/head_motion_generator
    │   └── util/                           # logger
    └── src/                                # 위 헤더에 대응하는 구현 (main.cpp 포함)
```

---

## 좌표계 / 운동학 컨벤션

### 단위

| 위치 | 단위 |
|---|---|
| 사용자 입력 (TCP) | 도 (degree) |
| JSON 설정 파일 (`motors.json`, `robot_poses.json`, `kinematics.json`) | 도 (degree) |
| `drum_coordinate.json` 의 `wrist_angle_deg` | 도 (degree, 로드 시 라디안 변환) |
| 코드 내부 (`q`, `qd`, 모든 관절각) | 라디안 (rad) |
| 작업공간 좌표 (`right_position`, `left_position`) | 미터 (m) |

### 좌표계

- 베이스 원점: 허리(waist) 회전축
- 어깨 간격: `link_length.waist` (0.520 m)
- 상완 길이: `link_length.upper_arm` (0.230 m)
- 하완 길이: `link_length.forearm` (0.200 m)
- 스틱 길이: `link_length.stick` (0.373 m)

값은 `kinematics.json` 참조.

### DH 컨벤션

- Modified DH (Craig) 사용
- 변환 행렬: `KinematicsSolver::dh_transform(a, alpha, d, theta)`
- 오른팔 / 왼팔 각각 6단 DH로 손 끝 좌표 계산

### 관절 방향

- 각 관절의 회전 방향은 `motors.json`의 `direction_sign` (±1.0)로 정의
- 모터 좌표 → 관절 좌표: `joint = motor * direction_sign + initial_joint_angle`

---

## 모터 구성

총 13개 관절. `motors.json`에 ID 순서대로 정의됨.

| ID | 이름 | 타입 | 모델 |
|---|---|---|---|
| 0  | waist            | TMotor     | AK10-9       |
| 1  | right_shoulder_1 | TMotor     | AK70-10      |
| 2  | left_shoulder_1  | TMotor     | AK70-10      |
| 3  | right_shoulder_2 | TMotor     | AK70-10      |
| 4  | right_elbow      | TMotor     | AK70-10      |
| 5  | left_shoulder_2  | TMotor     | AK70-10      |
| 6  | left_elbow       | TMotor     | AK70-10      |
| 7  | right_wrist      | MaxonMotor | DCX22L       |
| 8  | left_wrist       | MaxonMotor | DCX22L       |
| 9  | right_pedal      | MaxonMotor | DCX32L       |
| 10 | left_pedal       | MaxonMotor | DCX32L       |
| 11 | head_yaw         | Dynamixel  | XM430-W210-T |
| 12 | head_pitch       | Dynamixel  | XM430-W210-T |

### 지원 제어 모드

| 모터 | 제어 모드 |
|---|---|
| TMotor       | `POS` (SET_POS), `VEL` (SET_RPM + P 피드백), `SET_POS_SPD`, `SET_ORIGIN`, `CURRENT_BRAKE` |
| TMotor (MIT) | `MIT` — `tau = Kp(p_des − p) + Kd(v_des − qd) + t_ff`. **현재 기본 모드** |
| MaxonMotor   | `CSP` (Cyclic Sync Position), `CST` (Cyclic Sync Torque), `HMM` (Homing). `CSV` 미구현 |
| Dynamixel    | 위치 제어 (Profile Acceleration / Velocity + Goal Position) |

### 제어 모드 기본값

- **TMotor(0~6) = `MIT`** — `motors.json` 최상위 `"tmotor_mit"` 로 전환.
  `false` 면 기존 `VEL` 로 되돌아갑니다
- Wrist(7, 8) = `CSP` (단, 연주 중에는 `CST`로 전환)
- Pedal(9, 10) = `CSP`
- Head(11, 12) = Dynamixel 위치 제어 (`ControlMode::NONE`)

MIT 로 바꾼 이유는 학습이 모델링한 PD 법칙(`stiffness`/`damping`)을 실기에서 그대로
재현하기 위해서입니다. MIT 는 PhysX ImplicitActuator 와 식이 같아 `Kp`/`Kd` 를
그대로 옮길 수 있습니다.

**MIT 구간에서는 토크 과부하 차단이 관측만 하고 정지시키지 않습니다.**
남는 보호는 송신 전 급변 차단(30°)·관절 범위 검사와 모터 펌웨어의 토크 clamp 입니다.
이유는 `controller.cpp::safety_check_recv_tmotor` 주석 참조.

### 모터 통신

| 모터 | 물리 계층 | 프로토콜 |
|---|---|---|
| TMotor      | CAN 1Mbps                    | TMotor 독자 Servo / MIT 모드 |
| MaxonMotor  | CAN 1Mbps                    | CANopen (SDO / PDO) |
| Dynamixel   | UART 4.5Mbps (`/dev/ttyUSB0`) | Dynamixel Protocol 2.0 |

---

## 명령 프로토콜

### 패킷 형식

```
OPCODE|arg1|arg2\n
```

필드 구분자 `|`, 패킷 구분자 `\n`. Opcode는 대소문자를 구분하지 않습니다.

### Opcode 목록

| Opcode | 인자 | 설명 | 허용 상태 |
|---|---|---|---|
| `START`     | 없음                                            | 모터 토크 ON + home 자세. `Standby → Init` | Standby |
| `READY`     | 없음                                            | 고정 키 제거 완료 후 동작 허용 상태로 전환. `Init → Idle` | Init |
| `LOOK`      | `pan_deg`, `tilt_deg`                          | 머리 yaw / pitch 제어 | Idle |
| `GESTURE`   | `type`                                          | 제스처 (`nod` / `shake` / `wave` / `hi` / `hurray` / `happy`) | Idle |
| `MOVE`      | `motor_name`, `angle_deg`, `[move_time=3.0]`   | 개별 관절 이동 | Idle |
| `POSE`      | `pose_name`                                     | 사전 정의 포즈로 이동 (`home` / `ready` / `shutdown`) | Idle |
| `HIT`       | `target`                                        | 단일 드럼 타격 | Idle |
| `PLAY`      | `id`                                            | 악보 연주. `config/play_list.json`의 id로 악보/음원 선택 | Idle |
| `PAUSE`     | 없음                                            | 연주 일시정지. **재개 지점(마디) 저장** 후 ready 복귀 → `RESUME` 가능 | Playing |
| `RESUME`    | 없음                                            | 일시정지한 곡을 저장된 마디부터 재개 (음악 없이 무음 재개) | Idle |
| `PLAY_CTRL` | `stop` 또는 `speed`, `scale`                    | 연주 제어. `stop`=완전 중지(재개 지점 폐기), `speed`=속도 배율(0.5~2.0) | Playing |
| `GET_STATUS`| 없음                                            | 상태 조회. 응답: `STATUS\|<state>\|<q0..q12>\|<speed>\|<pause_valid>\|<pause_id>\|<pause_bar>` (`pause_valid=1`이면 RESUME 가능) | 모든 상태 |
| `QUIT` / `Q`| 없음                                            | shutdown 포즈 이동 후 시스템 종료 | 모든 상태 |

> `START` 와 `QUIT` 외의 명령은 `send_active`가 켜진 이후(=`START`로 첫 궤적이 적재된 이후)에만 처리됩니다.

### HIT target 목록

`snare`, `floor`, `mid`, `top`, `closed hihat`, `open hihat`, `ride`, `right crash`, `left crash`, `bass`

> 타깃 문자열은 코드의 `instrument_name_to_id`(`include/common/robot_config.hpp`) 키와 정확히 일치해야 합니다(공백 포함). 예: `closed hihat`, `right crash`. 일치하지 않으면 `Unknown target instrument`로 거부됩니다.

### 명령 예시

```
START                       # 토크 ON + home (이후 고정 키 제거)
READY                       # 고정 키 제거 후 동작 허용 상태로 전환
POSE|ready                  # ready 포즈로 이동
LOOK|0|10                   # 정면, 위쪽 10도 응시
LOOK|-30|0                  # 왼쪽 30도 응시
GESTURE|nod                 # 끄덕임
GESTURE|wave                # 손 흔들기
MOVE|right_wrist|45         # right_wrist를 45도로 이동 (기본 3.0초)
MOVE|right_wrist|45|1.0     # right_wrist를 45도로 1.0초에 이동
MOVE|waist|-30|2.0          # 허리를 -30도로 2초에 이동
HIT|snare                   # 스네어 1회 타격
HIT|closed hihat            # 클로즈드 하이햇 1회 타격 (타깃 문자열에 공백 포함)
PLAY|BF                     # play_list.json의 BF(BasicFillin) 연주
PAUSE                       # 연주 일시정지 (재개 지점 저장, PLAYING 중)
RESUME                      # 멈춘 마디부터 이어서 연주 (IDLE 중)
PLAY_CTRL|stop              # 연주 완전 중지 (재개 지점 폐기)
PLAY_CTRL|speed|1.2         # 연주 속도 1.2배
QUIT                        # shutdown 포즈 이동 후 종료
```

각도 단위는 모두 **도(degree)** 이며 내부적으로 라디안으로 변환됩니다.

### 사전 정의 포즈 (`robot_poses.json`)

| 포즈 이름 | 설명 |
|---|---|
| `init`     | 초기 자세. TMotor / Maxon(관절 0~10)은 `motors.json`의 `initial_joint_angle`과 일치 |
| `home`     | 연주 대기 자세 (`START` 시 이동, 고정 키 위치) |
| `ready`    | 준비 자세 |
| `shutdown` | 종료 전 안전 자세 |

---

## 로봇 상태 (RobotState)

상태는 `AppContext::robot_state`(초기값 `Standby`)에 보관되며, `BehaviorPlanner`가 명령을 처리하면서 전이시킵니다.

```
Standby ──START──▶ Init ── READY ──▶ Idle ──PLAY──▶ Playing
   │                 │                 │               │
   │                 │                 │   연주 종료    │
   │                 │                 ◀───────────────┘
   │                 │                 │
   └──────────── QUIT / shutdown 포즈 ──┴──────────────▶ ShuttingDown
```

| 상태 | 의미 |
|---|---|
| `Standby`      | 전원 ON, 통신 확인 완료, **토크 미인가** 대기 상태. `START` 대기 |
| `Init`         | 토크 ON, home 자세 유지. **고정 키 제거** 후 `READY` 대기 |
| `Idle`         | 동작 허용. 모든 동작 명령 수신 가능 |
| `Playing`      | 연주 중 (`PLAY` 진입 시) |
| `ShuttingDown` | 종료 절차. 새 명령 거부, ControlQueue 소진 후 시스템 종료 |

### 상태 전이 트리거

| 전이 | 트리거 | 처리 위치 |
|---|---|---|
| `Standby → Init`      | `START` 입력 (`handle_start`). home 자세로 이동시키고 토크 ON | `BehaviorPlanner` |
| `Init → Idle`         | `READY` 입력 (`handle_ready`). 고정 키 제거 완료 후 동작 허용 | `BehaviorPlanner` |
| `Idle → Playing`      | `PLAY` 입력 (`handle_play`) 또는 `RESUME` 입력 (`handle_resume`) | `BehaviorPlanner` |
| `Playing → Idle`      | 악보 소진 후 motion_queue가 비면 자동 복귀 (`schedule_idle_motion`). `PAUSE`/`PLAY_CTRL\|stop`도 잔여 모션 폐기 후 같은 경로로 복귀 | `MotionPlanner` |
| `* → ShuttingDown`    | `QUIT` 입력 또는 `POSE shutdown` | `BehaviorPlanner` |

> `START` / `QUIT` 외의 명령은 `send_active`가 켜진 이후(=`START`로 첫 궤적이 적재된 이후)에만 처리됩니다. 또한 동작 명령(`LOOK` / `GESTURE` / `MOVE` / `POSE` / `HIT` / `PLAY`)은 `Idle` 상태에서만 허용되며, 그 외 상태에서는 거부됩니다.

### 상태별 대기(filler) 모션

`MotionPlanner`는 명령이 없고 `motion_queue`가 비어 있을 때(`schedule_idle_motion`), 현재 상태에 맞는 대기 모션을 채워 넣어 제어 주기를 유지합니다. 대기 모션의 종류는 `MotionType` enum으로 구분됩니다.

| 상태 | 채워지는 `MotionType` | 동작 |
|---|---|---|
| `Init`         | `STANDBY` | 고정 키 제거 전 **현재 위치 유지** (자세를 붙들고 대기) |
| `Idle`         | `IDLE`    | 대기 동작 |
| `Playing`      | (악보 소진 시) `Idle`로 전이 후 `IDLE` | 연주 종료 처리 |
| `ShuttingDown` | 없음      | 대기 모션을 채우지 않고 ControlQueue 소진을 기다림 |

> `MotionType::STANDBY`는 `Init` 상태의 대기 모션 전용 타입입니다. `MotionType::IDLE`은 `Idle` / `Playing` 종료 시 사용됩니다.

---

## 연주(PLAY) 처리

### 악보 형식 (`data/scores/<name>.txt`)

탭(`\t`) 구분 텍스트. 첫 줄은 `bpm`, 마지막은 `end`, 사이는 타격 이벤트입니다.

```
bpm	100
<bar>	<beat>	<note_R>	<note_L>	<vel_R>	<vel_L>	<is_kick>	<is_closed_hihat>	...
...
end
```

| 열 | 의미 |
|---|---|
| `bar`              | 마디 번호 |
| `beat`             | 직전 이벤트로부터의 박자 간격 (예: 0.6 = 한 박) |
| `note_R` / `note_L`| 오른팔 / 왼팔 타격 악기 번호 (0 = 무타격) |
| `vel_R` / `vel_L`  | 오른팔 / 왼팔 타격 강도 |
| `is_kick`          | 베이스 드럼 (1/0) |
| `is_closed_hihat`  | 하이햇 닫음 (1/0) |

### 악기 번호 ↔ 좌표

악기 번호는 `drum_coordinate.json`의 `id`와 매칭됩니다.

| id | 악기 | id | 악기 |
|---|---|---|---|
| 1 | snare | 5 | hihat |
| 2 | floor | 6 | ride |
| 3 | mid   | 7 | crashR |
| 4 | top   | 8 | crashL |

각 악기는 오른팔/왼팔 각각의 `position`(m)과 `wrist_angle_deg`를 가집니다.

### 시퀀스 구성

`PLAY`는 다음 순서로 `MotionPrimitive` 시퀀스를 생성합니다.

1. `DRUM(START)` — 연주 시작 자세(스네어 대기)로 이동
2. `DRUM(PLAYING)` × N — 슬라이딩 윈도우로 악보를 구간별로 잘라 생성. 한 마디(100 bpm 기준 2.4초) 분량이 모이면 한 구간을 방출
3. `DRUM(END)` — home 자세로 복귀

---

## 궤적 프로파일

`TrajectoryGenerator`가 지원하는 보간 방식. `make_translate`의 기본값은 `COSINE`, `POSE`는 `TRAPEZOIDAL`.

| 프로파일 | 설명 |
|---|---|
| `COSINE`       | 사인 보간. 양 끝에서 속도 0, 부드러운 가속 / 감속 (기본값) |
| `CUBIC`        | 3차 다항식. 양 끝에서 속도 0 |
| `QUINTIC`      | 5차 다항식. 양 끝에서 속도·가속도 모두 0 |
| `TRAPEZOIDAL`  | 사다리꼴 속도 프로파일. 가속(25%) → 등속(50%) → 감속(25%) |

---

## 상태 플래그 (AppContext)

| 플래그 | 의미 |
|---|---|
| `running`     | 전체 종료 플래그. false가 되면 모든 스레드 루프 탈출 |
| `send_active` | `Controller::send_loop` 활성화 신호. `MotionPlanner`가 첫 궤적을 `ControlQueue`에 적재할 때 켜짐 |
| `recv_active` | `Controller::recv_loop` 활성화 신호. `send_active`와 동일 시점에 켜짐 |
| `robot_state` | 위 RobotState. 초기값 `Standby` |

---

## 안전 메커니즘

### TMotor 과부하 차단 — 모드에 따라 다릅니다

**서보 모드(`VEL`/`POS`)** — 수신된 `current_motor_current` 가 `current_limit` 을
**연속 5회 초과**하면 시스템 정지. 일시적 스파이크는 카운터가 리셋되어 무시됩니다(`Motor::cnt`).

**MIT 모드** — 토크가 `mit_torque_safety` 를 넘으면 **로그만 남기고 정지시키지 않습니다.**
이 검사는 제어 항이 아니라 감시이고(`tau` 식에 아무것도 더하지 않으므로 학습과 실기의
제어 법칙은 그대로 같습니다), 오발동하면 연주 중에 로봇이 멈춰 **학습에 없던 실패 양식을
추가**하게 됩니다. 임계값 자체가 피크 토크 추정치이고 펌웨어가 이미 거기서 clamp 하므로
오발동 위험이 큽니다.

남는 보호는 아래 셋입니다:

- 모터 펌웨어의 토크 clamp (하드웨어 보호는 원래 이쪽 담당)
- 송신 전 급변 차단 / 관절 범위 검사 (위치 기반이라 오발동이 적음)
- 수신 후 관절 범위 초과 → 즉시 정지

실기 로그로 정상 연주 중 최대 토크를 확인한 뒤, 여유를 두고 되살릴 수 있습니다.

### IK 실패 처리

- `KinematicsSolver::ik_solve()`가 실패를 반환하면 해당 모션을 폐기하고 시스템은 계속 동작
- 연주 중 IK 실패 시 해당 구간 생성을 중단함
- 실패 원인: 도달 불가능한 좌표, NaN / Inf, 관절 한계 초과

### 송신 전 안전 검사 (TMotor)

- **급변 차단**: 목표 위치와 현재 위치 차이가 **30도(약 0.524 rad)** 이상이면 해당 모터 송신 건너뜀
  (`POS_DIFF_LIMIT`. 정책 구간에도 그대로 걸립니다)
- **범위 검사**: `motors.json`의 `min_angle` / `max_angle`을 벗어나면 송신 건너뜀
- MaxonMotor / Dynamixel은 송신 전 검사 없음 (모터 자체 제한에 의존)

### 수신 후 안전 검사

- **TMotor 범위 초과** → 즉시 `running = false` (전체 정지)
- **Maxon 범위 초과** → `encodeShutdown` PDO 송신 후 전체 정지

복구 절차:

1. 시스템 종료 후 모터를 안전 범위로 수동 이동
2. CAN 포트 재기동 (USB 전원 차단 후 재연결)
3. `motors.json` 한계 값 검토 후 재실행

---

## 빌드 및 실행

### 빌드

최초 1회 — ONNX Runtime 을 받습니다 (sha256 고정):

```bash
bash drumrobot_server/lib/onnxruntime/fetch.sh
```

이후:

```bash
make
```

**검증 도구** (별도 빌드, `main.out` 에 포함되지 않음):

```bash
cd drumrobot_server
make ort-check      # ORT 링크·rpath, 그래프 입출력 규약, 추론 시간
make obs-check      # obs 조립 단위검사
make score-check    # 악보 파싱·양자화·세기변환·밀도
make golden-check   # sim 이 만든 값 vs C++ 이 만든 값 요소별 대조
```

도구는 저장소 최상위에서 실행합니다:

```bash
./drumrobot_server/bin/ort_check
./drumrobot_server/bin/golden_check
./drumrobot_server/bin/score_check drumrobot_server/data/policy/obs_constants.json BasicFillin
```

### 실행

TCP 서버로 실행됩니다 (포트 1951):

```bash
sudo ./drumrobot_server/bin/main.out
```

> **반드시 저장소 최상위에서 실행하세요.** 설정 파일 경로가 `drumrobot_server/config/…`
> 로 되어 있어, 다른 디렉터리에서 띄우면 `Failed to open config/kinematics.json` 이 뜨고
> **링크 길이가 0 으로 읽혀 FK 가 전부 0 을 반환합니다.**

또는 최상위 Makefile 사용:

```bash
make run
```

### TCP 클라이언트 (LLM 모드, 별도 터미널)

```bash
python3 drumrobot_client/main.py
```

호스트 `127.0.0.1`, 포트 `1951`로 연결됩니다.

---

## 참고 사항

- **CAN bitrate**: 1Mbps
- **CAN 포트 리셋**: `can_ports.json`에 등록된 머신에서 `uhubctl`로 USB 허브 전원을 껐다 켭니다. 미등록 머신은 리셋을 건너뜁니다.
- **Maxon 보간**: `send_loop`에서 5ms 구간을 1ms × 5 스텝으로 분할해 CSP 위치를 선형 보간 전송. 5ms 시점에 TMotor / Maxon / Dynamixel 동시 송신
- **`virtual_maxon_motor`**: 소켓당 Maxon 모터 1개를 대표로 선정해 Sync 프레임(0x80) 전송에 사용
- **Logger**: 런타임에 `drumrobot_server/log/`에 `log_MMDD_HHmm_{name}.csv` 생성 (motor / trajectory / motion_command)

---

## RL 정책

### 무엇을 대체하나

연주(`Playing`) 구간의 **팔 9관절(허리 0 · 양팔 1~6 · 양손목 7,8)** 만 정책이 소유합니다.

| | 소유자 |
|---|---|
| 팔 0~8, `Playing` 구간 | **정책** |
| 팔 0~8, 전이 궤적(`START`/`END`)·`Idle`·`POSE`·`HIT` | planner (기존 개루프) |
| 발 9,10 · 머리 11,12 | 항상 planner |

소유권은 전역 플래그가 아니라 **`ControlSetPoint::policy_owns_arm` 에 실려 전달**됩니다.
궤적은 약 100ms 앞서 생성되고 `send_loop` 은 나중에 소비하므로, 두 시점에 플래그를
따로 읽으면 경계에서 어긋나 팔에 `q=0` 이 나갈 수 있기 때문입니다.

### 학습 쪽과의 계약

`data/policy/obs_constants.json` 하나가 계약서입니다. **주기·게인·판정 상수·관절 순서가
전부 여기서 들어오므로, 학습 설정을 바꿔도 C++ 은 재빌드가 필요 없습니다** — 재export 만 하면 됩니다.

| 항목 | 값 |
|---|---|
| 관측 | 148차원 — `joint_pos(9) joint_vel(9) tip_pos(6) drum_pos(24) next_hits(66) hit_armed(16) arm_role(2) per_arm(16)` |
| 그래프 입력 | 9개 원시 버퍼 (**정규화는 그래프 내부**에서) |
| 위치 단위 | 허리 기준 미터 |
| 관절 순서 | obs 순서 ≠ 모터 id. `[0, 2, 1, 5, 3, 6, 4, 8, 7]` |
| 액션 | 관절 각속도. `q_target = q_now + a × action_scale × policy_dt` |
| 주기 | `POLICY_TICK_STRIDE`(3) × 5ms = **15ms (66.7Hz)** |

### 정책을 켜지 않는 조건 — 모두 개루프로 안전 강등

- `obs_constants.json` 없음 / 무효
- ONNX 세션 생성 실패, 그래프 입출력 이름·개수 불일치
- **주기 불일치** — `obs_constants.json` 의 `policy_tick_stride`·`policy_dt` 가
  `robot_config.hpp` 의 `ROBOT::` 상수와 다름
- 팔 모터 0~8 중 결번, 관절 한계 뒤집힘
- `tmotor_mit == false`

주기 불일치를 하드 가드로 막는 이유는, 그것이 **조용히 틀리고 로그에도 남지 않는**
종류의 오류이기 때문입니다. 정책이 학습한 것과 다른 간격으로 적분됩니다.

### 경계를 지키는 세 장치

| 장치 | 막는 것 |
|---|---|
| **seqlock** (`PolicyTarget`, `JointSnapshot`) | 찢어진 값 — 72바이트를 나눠 쓰는 사이 읽어 *실재하지 않는 자세*가 나가는 것. 읽는 쪽(실시간 루프)이 절대 막히지 않습니다 |
| **워치독** (`3 × STRIDE` = 9틱 = 45ms) | 낡은 값 — 슬롯이라 정책이 죽어도 마지막 값이 남아 모르고 지나가는 것 |
| **소유권 동봉** (`policy_owns_arm`) | 경계 어긋남 — 생성과 소비가 플래그를 따로 읽는 것 |

### 학습 저장소

정책은 별도 저장소에서 Isaac Lab 으로 학습합니다. 배포용 스크립트 세 개가 계약을 잇습니다:

| 스크립트 | 하는 일 |
|---|---|
| `export_policy.py` | 체크포인트 → `policy.onnx` + `obs_constants.json`. `--motors-json` 을 주면 **손목 CST 게인이 학습과 맞는지 대조**하고 어긋나면 중단합니다 |
| `dump_golden.py` | sim 을 exported onnx 로 굴려 `golden.json` 생성 |
| `dump_conventions.py` | 관절 순서·한계·리셋 자세 덤프 |

### 검증 상태

`golden_check` 가 sim 이 만든 값과 C++ 이 만든 값을 요소별로 대조합니다.
스케줄러 90여 줄과 재장전 상태머신을 손으로 옮긴 것이라, 실기 전에 잡는 것이 목적입니다.

| 버퍼 | 결과 |
|---|---|
| `joint_pos` · `joint_vel` · `drum_pos` · `arm_role` | **오차 0** |
| `tip_pos` · `hit_armed` · `per_arm_pos` · `per_arm_time` | 스틱 길이 12mm 차이에서 옴 (아래 참조) |

성능: obs 조립 p99 1.2 µs, ONNX 추론 p99 20 µs — **예산 15,000 µs 의 0.3%**.

---

## 알려진 미확정 사항

실기 검증 전이므로 아래는 확정되지 않았습니다.

### 스틱 길이 — 373mm 인가 385mm 인가

`kinematics.json` 의 `stick` 은 **0.373** 입니다(실기가 지금까지 연주해 온 값).
학습 쪽 `tip_offset` 은 0.385 입니다. 이 12mm 차이 하나가 골든 테스트의 남은
불일치를 전부 설명합니다 — 0.385 로 바꾸면 `tip_pos` 오차가 12.7mm→1.4mm,
`hit_armed` 불일치가 38→3(전부 step 0), `per_arm_*` 이 0 이 됩니다.

**어느 쪽이 물리적으로 맞는지는 실측으로만 정해집니다.** 실제가 373mm 인데 0.385 로
두면 개루프도 정책도 **자세와 무관하게 정확히 12.0mm 짧게** 칩니다(드럼 접촉 반경 130mm 의 9%).

확정 방법 — `HIT|snare` 타격 위치를 보거나, 손목 회전축 중심에서 스틱 타격점까지 실측.

### MIT 프로토콜 상수

원본 코드가 `P`·`KP`·`KD` 범위는 초기화해 두었고 `V`·`T` 는 비워 두었습니다.
그 빈 자리를 MIT 표준 관례로 채웠고, **확인되지 않았습니다.**

| 값 | 위험 |
|---|---|
| `mit_v_limit = 50` | 틀리면 `joint_vel` 이 잘못된 배율로 obs 에 들어갑니다. **어떤 안전 검사도 속도를 보지 않습니다** |
| `mit_t_limit` 65 / 25 | 토크 읽기가 틀어집니다 |
| `mit_kp_max = 500`, `mit_kd_max = 5` | 실제 범위와 다르면 **그 비율만큼 다른 게인이 걸립니다.** 어떤 코드 검사로도 잡히지 않습니다 |

**검증 방법** — 토크 인가 상태에서 팔을 0.05 rad 밀어 잡고 로그의 토크를 읽습니다.
`Kp_실효 = 토크 ÷ 0.05` 가 100 이면 세 상수가 한 번에 검증됩니다.

### 실기에서 아직 돌지 않은 것

MIT 송수신 경로 전체 · MIT 진입 절차 · 게인 램프 · ERPM→rad/s 변환 ·
소유권 전환 · 워치독 · 정책 구동. **모두 코드 수준 검증만 되어 있습니다.**

---

## 실기 브링업 절차

MIT 모드는 **실기에서 한 번도 돌아간 적이 없습니다.** 정책부터 켜지 않고
**서보 → MIT → 연주 → 정책** 네 세션으로 올립니다.

| 세션 | 설정 | 하드웨어 | 목적 |
|---|---|---|---|
| **1** | `tmotor_mit: false` · 정책 OFF | 모터 1개 (벤치) | 기존 동작 회귀 확인 |
| **2** | `tmotor_mit: true` · 정책 OFF | 모터 1개 (벤치) | MIT 첫 가동 + **토크 측정** |
| **3** | `tmotor_mit: true` · 정책 OFF | 전 모터 (로봇) | MIT 개루프 연주 + **타격 위치** |
| **4** | `tmotor_mit: true` · 정책 ON | 전 모터 (로봇) | 정책 가동 |

> 세션마다 바뀌는 파일은 **`config/motors.json` 2번째 줄 하나**뿐입니다.
> 정책은 파일 이름으로 켜고 끕니다.

### 공통 주의

- **모터를 단단히 고정하세요.** 손으로 미는 측정이 있습니다
- 중단: `QUIT` → 안 되면 **전원 차단**. 스위치를 손 닿는 곳에
- MIT 모드에서는 **토크 초과 차단이 로그만 남기고 정지시키지 않습니다**

---

### 최초 1회 — 새 머신에서 시작할 때

```bash
git clone https://github.com/KIST-delight-robotics/phill_drum_RL.git
cd phill_drum_RL

# ONNX Runtime 받기 (네트워크 필요, ~28MB, sha256 검증)
bash drumrobot_server/lib/onnxruntime/fetch.sh

make
mkdir -p drumrobot_server/log

# 세션 1~3 동안 정책을 꺼둡니다
mv drumrobot_server/data/policy/policy.onnx \
   drumrobot_server/data/policy/policy.onnx.off
```

**오프라인 머신이면** — 다른 머신에서 받은 `drumrobot_server/lib/onnxruntime/{include,lib}`
두 디렉터리를 복사하세요. `.gitignore` 에 있어 git 으로는 가지 않습니다.

**hostname 확인** — `config/can_ports.json` 에 등록돼 있어야 CAN 포트 USB 리셋이 됩니다.
없으면 `Unrecognized hostname ... skip CAN reset` 이 뜨고 그대로 진행합니다(치명적이지 않음).

---

### 세션 1 — 서보 모드 · 모터 1개

기존 코드와 같은 상태입니다. 이식이 기존 동작을 깨지 않았는지 봅니다.

**고칠 파일** — `drumrobot_server/config/motors.json` 2번째 줄

```json
"tmotor_mit": false,
```

**터미널 A** — 모터를 고정하고 전원을 켠 뒤

```bash
cd <저장소 최상위>
make run
```

기대 출력:

```
[Robot] TMotor 제어 모드: VEL
[Robot] --------------> CAN NODE ID 4 Connected. Joint 4: right_elbow
[Robot] CAN NODE ID 0 Not Connected. ...        ← 나머지는 이게 정상
[Robot] TMotor Set Zero
[main] 정책을 켜지 못했습니다. 개루프로 연주합니다.
```

**터미널 B**

```bash
cd <저장소 최상위>
python3 drumrobot_client/main.py
```

```
START                       # 토크 ON. 모터는 제자리 유지 (안 움직이는 게 정상)
READY
MOVE|right_elbow|100|3.0    # 3초에 걸쳐 10도
MOVE|right_elbow|90|3.0     # 원위치
QUIT
```

**통과 조건** — 연결한 모터가 `Connected`, `START` 후 조용히 버팀,
`MOVE` 가 부드럽게 이동, `control_queue underflow` 반복 없음.

> `right_elbow`(id 4) 같은 **팔 모터**를 권합니다. TMotor 7개 중 6개가 AK70-10 입니다.

---

### 세션 2 — MIT 모드 · 모터 1개

**MIT 가 실기에서 도는 첫 순간입니다.** 이 세션의 목적은 토크 측정입니다.

**고칠 파일** — `motors.json` 2번째 줄 (되돌립니다). 재빌드 불필요.

```json
"tmotor_mit": true,
```

**터미널 A**

```bash
make run
# [Robot] TMotor 제어 모드: MIT      ← MIT 로 떠야 합니다
```

**터미널 B**

```
START      # MIT 진입 + 게인 램프 0→1 (0.5초)
```

여기서 **아무것도 하지 말고 10초쯤 지켜봅니다.** 램프 중에는 목표가 실측 위치로
고정되므로 안 움직이는 것이 정상입니다.

> **즉시 전원 차단** — 고주파 떨림·윙윙거림(게인이 예상보다 크게 걸린 것),
> 모터가 스스로 움직이려 함, `토크 한계 근접` 로그가 계속 뜸.

#### 토크 측정 — 미검증 상수 3개를 한 번에

1. 모터 출력축을 손으로 **약 3도(0.05 rad) 밀어** 5초쯤 잡고 있는다
2. 손을 떼고 `QUIT`

```bash
awk -F, '$1==4 && $2==3 {print $5, $6}' \
  drumrobot_server/log/log_*_motors.csv | sort -g -k1 | tail -5
#         ↑ err[rad]  ↑ torque[N·m]      $1 은 모터 id, $2==3 은 MIT
```

`Kp_실효 = 토크 ÷ err` — 둘 다 출력축 기준이라 기어비 환산이 필요 없습니다.

| err 0.05 rad 에서 토크 | Kp_실효 | 판정 |
|---|---|---|
| **약 5.0 N·m** | 100 | **정상** — 게인·토크 디코딩 둘 다 맞음 |
| 약 10 N·m | 200 | 게인 2배 — `mit_kp_max` 가 실제와 다름 |
| 약 2.5 N·m | 50 | 게인 절반 |
| 0 또는 이상값 | — | `mit_t_limit` 토크 디코딩 문제 |

이 측정 하나로 **`mit_kp_max` · `mit_kd_max` · `mit_t_limit` 이 동시에 검증됩니다.**
어긋나면 그 비율만큼 `motors.json` 의 `mit_kp` 를 보정합니다.

속도(`mit_v_limit`)도 같은 로그에서 봐 두세요 — 손으로 천천히 돌렸을 때 값이
상식적인지. **어떤 안전 검사도 속도를 보지 않으므로** 눈으로 보는 것이 유일한 방법입니다.

---

### 세션 3 — MIT 로 기존 연주 · 로봇 전체

**설정 변경 없습니다.** 세션 2 상태 그대로, 모터를 전부 연결하고
**고정 키를 꽂은 채** 전원을 켭니다.

```bash
make run     # 13개 관절 전부 Connected 여야 합니다
```

```
START            # 토크 ON — 터미널 A 에 키 제거 안내가 나옵니다
```

> **여기서 고정 키를 전부 제거합니다.** 토크가 자세를 붙들고 있을 때 뽑아야 합니다.
> `Init` 상태에서는 팔을 움직이는 명령이 전부 거부되니 안전합니다.

```
READY
POSE|ready
HIT|snare        # ★ 타격 위치를 눈으로 확인 (아래)
PLAY|BF
PLAY_CTRL|stop   # 이상하면 즉시
QUIT
```

**통과 조건** — 세션 1(서보)과 같은 소리·같은 자세, **팔이 처지지 않음**,
최대 토크가 `mit_torque_safety`(팔 24 N·m)에 여유 있게 못 미침.

```bash
awk -F, '$2==3 {t=($6<0?-$6:$6); if(t>m[$1]) m[$1]=t}
     END{for(i in m) printf "  id %s  최대 %.2f Nm\n", i, m[i]}' \
  drumrobot_server/log/log_*_motors.csv
```

#### HIT|snare 타격 위치 — 스틱 길이가 여기서 확정됩니다

| 결과 | 뜻 | 조치 |
|---|---|---|
| 중앙을 잘 침 | 373 이 맞다 | **유지.** 학습 `specs.py` 의 `tip_offset` 을 0.373 으로 고쳐 재학습 |
| 12mm 깊게/넘어가게 침 | 385 가 맞다 | `kinematics.json` 73줄을 0.385 로. 재학습 불필요 |

---

### 세션 4 — 정책 ON · 로봇 전체

세션 1~3 이 전부 통과한 뒤에만 진행합니다.

```bash
mv drumrobot_server/data/policy/policy.onnx.off \
   drumrobot_server/data/policy/policy.onnx

make run
```

**기동 시 이 줄을 반드시 확인하세요:**

```
[PolicyRunner] 준비 완료 — 주기 15ms (66.6667Hz), stride 3, 모델 ...
```

`[main] 정책을 켜지 못했습니다` 가 나오면 **바로 위에 이유가 찍힙니다** —
주기 불일치 / 팔 모터 결번 / 세션 생성 실패 / MIT 아님.

```
START → 고정 키 제거 → READY → POSE|ready

PLAY|BF
PLAY_CTRL|speed|0.5      # 절반 속도가 안전합니다

# [TrajectoryGenerator] 팔 0~8 소유권을 정책에 넘겼습니다
#    ... 15초 지켜본 뒤 ...
PLAY_CTRL|stop
# [TrajectoryGenerator] 팔 0~8 소유권을 되받았습니다
QUIT
```

**곡을 끝까지 들을 필요 없습니다.** 15초면 소유권 **인계와 반납 두 경계**를
다 시험합니다 — 가장 위험한 순간들입니다.

속도 배율은 정책의 `next_hits` 에도 반영되어 타격 간격이 2배로 벌어집니다.
밀도 위반이 사라지고 팔도 천천히 움직입니다.

**멈출 신호** — `[Controller] 정책 워치독`, `[PolicyRunner] obs 생성 실패` 반복,
팔이 드럼이 아닌 곳으로 감, **소유권 전환 순간에 자세가 튐**.

종료 후 터미널에 요약이 찍힙니다:

```
[PolicyRunner] N주기  평균 xx us  p50 xx  p99 xx  최대 xx us  (예산 15000us)
```

최대가 15,000 µs 를 넘지 않으면 정상입니다 (예상 ~20 µs).

---

### 곡 선택

| id | 악보 | 길이 | 105ms 미만 구간 | 최소 간격 | 오디오 |
|---|---|---|---|---|---|
| **BF** | BasicFillin | 144 s | 48개 | **90 ms** | — |
| BI | BabyINeedYou | 138 s | 30개 | 90 ms | BI.wav |
| DS | DrumSolo | 125 s | 8개 | 60 ms | — |
| TI | ThisIsMe | 243 s | 2개 | 75 ms | TIM.wav |
| WS | WhySo | 389 s | 10개 | **45 ms** | — |
| TY | ToYou | 554 s | 4개 | 90 ms | — |

학습은 타격 간격을 **105ms 미만으로 만들지 않습니다**(`min_gap = 2×hit_window_step+1`).
그보다 촘촘하면 정책이 본 적 없는 밀도입니다. 모든 곡에 조금씩 있고,
**개수보다 얼마나 촘촘한가**가 중요합니다.

**`BF` 를 권합니다** — 위반 48구간이 전부 90ms 로 가장 완만하고(하한에서 1스텝 부족),
오디오가 없어 변수가 하나 줄어듭니다.

**`M1` 은 쓰지 마세요** — `score` 와 `midi` 를 둘 다 갖고 있어, 정책은 MIDI 를
발·머리는 txt 악보를 읽습니다(`behavior_planner.cpp:533`). 두 악보가 다르면
**팔과 발이 서로 다른 음악을 연주합니다.** MIDI 경로 시험 전용 항목입니다.

---

### 벤치(모터 1개)에서 알아둘 것

**되는 명령**

| 명령 | |
|---|---|
| `START` `READY` `QUIT` | 상태 전이 |
| `MOVE\|<관절>\|<도>\|<초>` | **벤치 주력** |
| `POSE\|ready` | 연결된 모터만 움직임 |
| `HIT` `PLAY` | ❌ IK 가 양팔 전체를 풀어야 함 — 무의미 |

**왜 모터 하나로 되나** — 미연결 모터는 `robot.initialize()` 가 맵에서 제거하고,
송신 게이트(`all_tmotors_received`)는 맵을 순회하므로 연결된 것만 기다립니다.
다이나믹셀 포트가 없어도 해당 모터를 지우고 계속 진행합니다.

**왜 `START` 해도 안 움직이나** — `home` 이 모든 TMotor 의 `initial_joint_angle` 과
같고, `Set Zero` 가 시동 위치를 그 값으로 잡습니다. 즉 "지금 자리를 유지하라"가 됩니다.

**안 움직일 때 — 가동 범위** — `Set Zero` 가 **시동 순간의 물리 위치**를 기준으로
잡습니다. `right_elbow` 범위가 `0~140.1도` 이고 시동 위치가 `90도` 가 되므로,
실제로는 **시동 위치에서 −90°~+50°** 안에서만 움직입니다. 밖을 명령하면
`범위 초과 차단` 로그가 뜨고 조용히 건너뜁니다.

---

### 로그 컬럼 — `drumrobot_server/log/log_MMDD_HHmm_motors.csv`

| # | 이름 | 뜻 |
|---|---|---|
| 1 | `id` | 모터 id (0 허리 · 1~6 팔 · 7,8 손목 · 9,10 발) |
| 2 | `mode` | 1 위치 · 2 속도 · **3 MIT** |
| 3 | `desired` | 목표 위치 [rad, 출력축] |
| 4 | `actual` | 실측 위치 [rad, 출력축] |
| 5 | `err` | `desired − actual` — **토크 측정에 쓰는 값** |
| 6 | `current/torque` | MIT 는 **토크 [N·m]**, 서보는 전류 [A] — **의미가 다릅니다** |
| 7 | `input` | MIT 는 게인 램프값 0~1 |


---

## 라이선스 및 외부 의존성

> ⚠️ 현재 저장소에 LICENSE 파일이 없습니다. 공개 전 라이선스 명시를 권장합니다.

| 라이브러리 | 위치 | 라이선스 |
|---|---|---|
| nlohmann/json   | `drumrobot_server/lib/nlohmann/json.hpp` | MIT        |
| Dynamixel SDK   | `drumrobot_server/lib/dynamixel_sdk/`    | Apache 2.0 |