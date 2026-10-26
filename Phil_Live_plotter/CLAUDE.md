# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 이 프로젝트

드럼로봇 제어 코드(**Phil**, 별도 C++ 프로젝트)의 SocketCAN 버스를 **옆에서 엿듣기만 하는**
라이브 플로터. 버스에 흐르는 프레임을 디코딩해서 모터별 위치/속도/토크의
**명령(TX)과 피드백(RX)**을 브라우저에 그린다. candump 로그 파일 전체 구간 분석 모드도 있다.

### 넘지 말아야 할 선

사용자가 명시한 목표는 **"로봇 제어 코드에 어떠한 간섭도 없이 CAN 만 보고 플로팅"** 이다.

- 로봇 코드(`Phil/`)를 **수정하지 않는다.** 설정 파일도 읽기 전용으로만 본다.
- CAN 버스에 **아무것도 송신하지 않는다.**
- CAN 인터페이스를 올리거나 내리지 않는다 (제어 프로세스/셋업 스크립트 담당).
- 로봇의 `make run` 에 플로터를 끼워넣는 건 사용자가 나중에 직접 하기로 했다.
  준비물로 `scripts/launch_terminal.sh` 만 만들어 뒀고, Makefile 은 건드리지 않았다.

## 실행

```bash
pip3 install -r requirements.txt          # websockets 만 필요

./run.sh                                  # 백엔드 + GUI (플로터만. 로봇은 안 띄운다)

# 개별 실행
python3 backend/can_bridge.py             # can*/vcan* 자동 탐색, ws://127.0.0.1:8765
python3 backend/can_bridge.py -i can0,can1        # 인터페이스 지정
python3 backend/can_bridge.py --replay logs/x.log --replay-speed 0.5   # 기록 재생
python3 -m http.server 8099               # 별도 터미널에서 GUI (file:// 로 열면 config fetch 실패)
```

GUI: `http://localhost:8099/index.html`
URL 해시로 뷰/파일 지정 가능: `#overview`, `#view=overview&file=logs/candump-....log`

백엔드가 안 붙으면 프런트엔드는 자동으로 목업 데이터로 돈다 (배지에 `MOCK DATA`).

## 구조

```
config/motors.json   ← 플로터 전용 설정 (모터 목록은 여기 없다. 아래 참고)
backend/motor_config.py   로봇 설정을 읽어와 내부 형식으로 변환 + 프로토콜 상수
backend/protocol.py       CAN 프레임 디코더 + 방향(tx/rx) 판별
backend/can_bridge.py     SocketCAN 수신 스레드 → asyncio 큐 → WebSocket, candump 제어, 로그 분석
static/app.js             차트/상태 관리, WS 클라이언트, 목업 생성기
run.sh                    플로터(백엔드+GUI) 실행
scripts/launch_terminal.sh  플로터를 새 터미널 창으로 (make run 연동용, 아직 미사용)
Phil/, DrumRobot/         참조용 로봇 코드. DrumRobot 은 구버전이고 Phil 이 최신
```

**모터 목록의 정본은 로봇 코드의 `config/motors.json` 이다.** `motor_config.py` 가 그걸 읽어
(`PLOTTER_ROBOT_CONFIG` 환경변수 > `robotConfigPath` > `robotConfigSearch` 순으로 탐색)
내부 형식으로 변환한다. 못 찾으면 `fallbackMotors` 로 떨어진다.
로봇 설정은 웹 루트 밖에 있을 수 있으므로 **브라우저는 fetch 하지 못한다.** 그래서 백엔드가
접속 시 `{type:"config", motors:[...]}` 로 내려주고, 프런트엔드는 그걸로 목록을 갈아끼운다
(로컬 `fallbackMotors` 는 백엔드가 없을 때만 쓴다).

데이터 경로가 하나로 모이는 게 핵심 설계다.

- 백엔드: 라이브 수신 / 로그 재생 / 로그 파일 분석 셋 다 `protocol.classify()` 하나를 거친다.
- 프런트엔드: 목업이든 실데이터든 전부 `ingestFrame()` 하나로 들어간다.

디코딩 로직을 고칠 일이 생기면 `protocol.py` 한 곳만 보면 된다.

## 프로토콜 디코딩 — 여기가 이 프로젝트의 핵심 지식

`Phil/drumrobot_server/src/hardware/motor_codec.cpp` 의 비트 패킹을 파이썬으로 그대로 옮긴 것이다.
값이 이상하면 먼저 그 C++ 원본과 대조할 것. (TMotor Servo/MIT 인코딩은 구버전 DrumRobot 과 동일하다.)

버스를 엿듣기만 하므로 **프레임이 명령인지 피드백인지는 프로토콜 규칙으로 추정**해야 한다:

| | 명령(tx) | 피드백(rx) |
|---|---|---|
| Servo | 확장 프레임, `can_id = nodeId \| packetId<<8` | 확장 프레임, `can_id = nodeId \| 0x29<<8` (STATUS), 8바이트 |
| MIT | 표준 프레임, `can_id == nodeId`, 8바이트 | `data[0] == nodeId`, 응답 ID 는 nodeId 또는 마스터 ID(0), 8바이트 |
| Maxon | `0x300 + nodeId` = 목표 위치(디코딩함), 그 외 PDO/SDO 는 raw | `0x180 + nodeId` (TPDO1) |

Servo 는 명령·피드백이 **둘 다 확장 프레임**이라 packet id 로 가른다. `0x29`(STATUS)만 피드백이고
나머지는 명령이다. 로봇 C++ 원본에는 이 구분이 없다 — 자기가 보낸 프레임은 자기가 아니까
`can_id & 0xFF` 로 모터만 찾는다. 엿듣는 쪽에서만 필요한 규칙이다.

MIT 는 명령과 피드백이 **같은 can_id + 같은 8바이트**로 오는 모터가 있어서(실장비 확인)
ID 로 못 가르고 `data[0] == nodeId` 로 피드백을 먼저 골라낸다(`classify()` 3번 분기).
명령의 data[0] 은 목표 위치 상위 바이트라 범위 끝(-12.5 rad 근처)이 아니면 nodeId 와 안 겹친다.
또 MIT 명령은 표준 프레임이라 Servo 피드백을 표준 프레임으로 주는 펌웨어와는 구분이 안 된다
(4번 분기가 그 폴백이다). 그래서 GUI에서 고른 모터별 모드가 해석 방향을 결정한다.
프런트엔드가 모드를 바꿀 때마다 `set_modes` 컨트롤 메시지로 백엔드에 알려주며,
백엔드는 이걸 모르면 디코딩을 할 수 없다.

주의할 점:
- **Servo STATUS packet id(0x29)는 `config/motors.json` 의 `servoStatusPacketId` 에 있다.**
  실장비 로그(node 5 → `0x2905`)로 확인했다. 펌웨어가 다르면 코드가 아니라 이 값을 고칠 것.
- **MIT 피드백 응답 ID 는 모터마다 다르다** (2026-09-29 로그). 대부분 자기 nodeId 로(명령과 같은 ID),
  node 3 만 마스터 ID(0x000, `config/motors.json` 의 `mitMasterId`)로 회신했다. 8바이트이고
  data[6] 온도(+40 오프셋), data[7] 에러. node 3 은 뒤 2바이트가 0 이라 온도를 안 찍는다.
  ID 0x000 은 CANopen NMT 와 겹쳐서 `_looks_like_nmt()` 로 걸러낸다.
- 이 로그에서 MIT 모드 허리는 **0x011** 로 명령/응답했다 (설정은 0x00, 서보 모드는 0x900 = node 0).
  설정에 없는 ID 라 미매핑으로 남는다.
- FF×7 + FC/FD/FE 는 MIT 진입/종료/영점 특수 명령이라 값으로 디코딩하지 않는다.
- **Servo 속도는 버스 위에서 ERPM(전기적 RPM)이다.** 명령·피드백 둘 다 그렇다.
  `controller.cpp` 가 `erpm = joint_vel * pole * gear_ratio * 60/2pi` 로 만들어 보내므로,
  `erpm_to_rad_s()` 가 그걸 그대로 뒤집어 rad/s 로 되돌린다. **기어비가 모터마다 다르니
  (AK10_9=9, AK70_10=10) 반드시 로봇 설정의 `gear_ratio` 를 쓸 것.**
  C++ 의 `decodeFeedback` 주석은 속도를 "radians per second" 라고 해놨지만 **틀렸다.**
  환산 없이 ERPM 이 그대로 담긴다. 로봇은 이 값을 안 쓰는 듯해서 문제가 안 됐을 뿐이다.
- **VEL 모드 명령은 순수 속도 지령이 아니다.** `control_input = erpm + control_gain * err`
  라서 위치 오차 보정항이 섞여 있다. 명령선과 피드백선이 딱 안 겹치는 건 정상이다.
- **Servo 모드의 토크 채널은 사실 전류[A]다.** C++ 원본도 Kt를 안 곱한다. MIT 모드는 기종 Kt를
  아는 경우에만 N·m로 환산한다.
- 로봇의 현재 제어 경로는 **Servo 모드만** 쓴다. MIT 코덱은 코드에 있지만 제어 루프에서
  호출되지 않는다. 그래서 기본 모드가 servo다.
- **구버전(DrumRobot) 대비 바뀐 것** — 새 코드를 볼 때 헷갈리기 쉬운 부분:
  손목(0x07, 0x08)이 TMotor → **Maxon** 으로 바뀌었고, Maxon 피드백이 SDO(0x580) 가 아니라
  **PDO(0x180)** 다. `maxonForTest` 는 없어졌고 페달은 0x0A/0x0B, 머리 2축은 Dynamixel(CAN 아님)이라
  목록에서 빠진다. 이 값들은 전부 로봇 설정에서 읽으므로 코드에 박아두지 말 것.
- Servo 위치 명령(SET_POS)에는 속도/토크가 없다. 그래서 `decoded` 는 부분 필드만 담을 수 있고,
  프런트엔드는 없는 값을 건너뛴다 (차트에 "이 프레임에 해당 값 없음" 표시).

## 성능 관련 설계 (실제 로봇 속도 = 5ms 주기 × 12모터 ≈ 초당 5천 프레임)

- 백엔드는 20ms 동안 프레임을 모아 `{type:"batch"}` 로 묶어 보낸다. 개별 전송하면 못 버틴다.
- 프런트엔드는 그래프에 찍는 점만 50Hz로 솎아낸다(`PLOT_MIN_INTERVAL_SEC`). 프레임 로그와
  현재값은 전부 반영한다.
- 로그 파일 분석은 구간마다 **최소/최대 두 점**만 남긴다. 단순 샘플링과 달리 타격 순간의
  토크 스파이크가 살아남는다. 18MB/43만 줄 → 2.5초 디코딩, 브라우저로는 1.3MB 전송.
- 파일 분석 모드에서는 주기적 재렌더링을 멈춘다. 수십만 점을 150ms마다 다시 그리면 브라우저가 멈춘다.

## CAN 인터페이스 여러 개 다루기

인터페이스는 4개까지 생기고, 테스트할 때 일부만 연결하는 경우가 흔하다. 그래서:

- 모터 ↔ 인터페이스 고정 매핑이 없다. 있는 인터페이스를 전부 열어 스레드로 동시에 읽고,
  프레임의 노드 ID로 모터를 찾아 합친다 (DrumRobot 의 `setMotorsSocket()` 도 같은 발상이다).
- `interface_scanner()` 가 5초마다 다시 훑어서 **실행 중에 생긴 인터페이스**도 잡는다.
  `-i` 로 명시했을 때는 스캔하지 않는다.
- `loggable_interfaces()` 로 살아있는 것만 candump 에 넘긴다. 인자 중 하나라도 없는
  인터페이스면 candump 가 통째로 실패하기 때문이다.

**제약**: 서로 다른 버스의 같은 노드 ID를 구분하지 못한다 (합쳐진다). CAN 에서 ID 는 버스마다
독립이므로 원칙적으로 가능한 상황인데, 지금 설정은 0~11 이 유일해서 문제되지 않는다.
버스별로 ID 를 재사용하게 되면 `motors.json` 에 인터페이스 필드를 추가하고
`classify()` 에 그 조건을 넣어야 한다.

## candump 자동 로깅에서 걸렸던 것들

`log_supervisor()` 가 인터페이스가 살아있는 한 candump 를 계속 살려둔다. 여기서 실제로 물린 것들:

- **트래픽이 없는 candump 는 SIGTERM 으로 안 죽는다.** select() 에 블록된 채 종료 플래그를
  확인하지 못한다. 그래서 `stop_logging` 은 SIGTERM → 1초 대기 → SIGKILL 로 올라가고,
  `PR_SET_PDEATHSIG` 도 SIGTERM 이 아니라 **SIGKILL** 로 걸어야 한다.
- 그 PDEATHSIG 가 없으면 백엔드를 SIGKILL 했을 때 candump 가 고아로 남아 계속 쌓인다.
- asyncio 기본 동작으로는 SIGTERM 에 `finally` 를 안 타므로, 시그널 핸들러로 이벤트를 깨워서
  정리 경로를 타게 해뒀다.
- candump 는 트래픽이 없어도 파일부터 만든다. 그래서 시작할 때 10분 넘은 0바이트 로그를 지운다.

## 이 환경에서 알아둘 것

- CAN 어댑터는 PEAK **pcan_usb**, 비트레이트 1 Mbit/s (`sudo ip link set can0 up type can bitrate 1000000`).
- **passwordless sudo가 없다.** 링크를 올리고 내리는 건 사용자가 직접 해야 한다.
- **USB 어댑터를 뽑았다 꽂으면 인터페이스가 재열거되고(ifindex 변경) 비트레이트 설정이 날아간 채
  DOWN 으로 올라온다.** 그래서 수신 스레드는 매 재시도마다 ifindex가 아니라 **이름으로 다시 bind** 한다.
- 링크가 내려가면 `recv()` 는 timeout이 아니라 `ENETDOWN` 을 던진다. timeout은 "링크는 살아있고
  트래픽만 없음"이라는 뜻이다. 이 구분으로 링크 상태를 판정한다.
- 하드웨어 없이 테스트하려면 `vcan0` 를 쓴다 (이 머신에 이미 있음). SocketCAN raw 소켓은 root 불필요.
- 다른 프로세스가 보낸 프레임도 로컬 에코로 이 플로터에 보인다. 그래서 제어 프로세스의 명령 프레임을
  볼 수 있다. 단, 하드웨어에서는 에코가 TX 완료 후에 오므로 **버스에 ACK할 노드가 없으면 안 보인다**.

**실제 버스로 프레임을 절대 보내지 말 것.** 모터가 물려 있으면 명령 프레임이 관절을 실제로 움직인다.
반대로 버스에 다른 노드가 없으면 ACK를 못 받아 어댑터가 bus-off로 갈 수 있다.

## 검증 상태

- 디코더는 C++ 인코딩 → 파이썬 디코딩 왕복 테스트로 확인됨 (Servo 명령/피드백, MIT 명령/피드백, Maxon).
  **다만 이 테스트는 믿을 게 못 된다.** 피드백에는 C++ 인코더가 아예 없어서(모터가 보내는 거니까)
  기대값을 우리가 가정해서 만들고 그 가정을 되돌려 받는 자기충족 테스트가 된다. 실제로 아래
  실장비 버그 두 개를 이 테스트가 전부 통과시켰다. **피드백 쪽은 실장비 로그로만 확인된다.**
- vcan0 로 200Hz × 9모터 종단간 확인, candump 로깅/재생/파일 분석 확인.
- 실제 can0(pcan_usb) 하드웨어: 링크 붙기·끊김·자동 재연결까지 확인 (USB 재열거 포함).
- candump 자동 로깅: 자동 시작, 강제 종료 후 자동 복구, 백엔드 종료 시 동반 정리 확인.
- `run.sh`: 기동/중복 방지/Ctrl+C 정리 확인. 다만 **새 터미널 창이 뜨는 경로는 미검증**
  (GUI 창을 임의로 띄우지 않았다). ssh 등 폴백 경로는 확인함.
- Phil 설정 연동: 로봇 설정을 읽어 모터 11개(손목 Maxon 포함)를 구성하고, 백엔드가 GUI 로
  내려주는 것까지 vcan0 로 확인 (Maxon PDO 피드백/목표 위치 명령 디코딩 포함).
- 실제 모터 **1대(node 5, AK70_10)** 트래픽으로 확인 (2026-09-08). 정지 상태 6.6만 프레임 +
  실제 구동 9.3만 프레임이 미매핑 0으로 디코딩된다. 여기서 버그 두 개가 드러났다:
  1. **서보 피드백은 표준 프레임이 아니라 확장 프레임(`0x2905`)** 이다. 그 전에는 피드백이
     packet id 41 짜리 명령으로 흡수돼 `*Fb` 계열이 통째로 비었다.
  2. **속도 피드백을 ERPM 그대로 찍고 있었다.** 명령만 rad/s 로 환산돼 축이 200배 어긋나
     명령선이 0 에 눌려 안 보였다. 기어비까지 넣어 고친 뒤 명령 vs 피드백
     **상관계수 0.9963**, 범위도 -6.92~4.98 vs -6.92~5.05 rad/s 로 겹친다.
- **여러 모터 동시 구동과 Maxon(손목/페달) 실트래픽은 아직 미검증.** 로봇을 돌릴 기회가 생기면
  사이드바의 `N/N 프레임 디코딩` 카운터와 미매핑 ID 경고부터 볼 것.
- 인터페이스 스캐너가 **새 인터페이스를 잡는 경로도 미검증** (root 없이 vcan 을 못 만들어서).

## 브링업이 안 될 때

프레임이 안 보이면 조용히 실패하지 않도록 진단이 붙어 있다. 사이드바에 인터페이스 상태와
디코딩 카운터가 뜨고, 매핑 안 되는 CAN ID가 있으면 상위 목록을 보여준다.
원인은 대개 셋 중 하나다: 링크 DOWN / `config/motors.json` 의 nodeId 불일치 / 모드(Servo·MIT) 불일치.

## 코딩 관례

- 주석과 UI 문구는 한국어.
- 모터 ID·기종을 코드에 하드코딩하지 말 것. 로봇 설정에서 읽는다.
  프로토콜 상수는 `config/motors.json` 으로 간다.
  (설정을 고치면 백엔드 재시작 + 브라우저 새로고침 필요)
- **로봇 코드(`Phil/`, `DrumRobot/`)는 읽기 전용.** 수정하지 말 것.
