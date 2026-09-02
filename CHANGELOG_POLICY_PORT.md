# 정책 이식 변경 기록

> 대상 `drumrobot_server` · 기준 HEAD `8298e3b`
> 설계 근거는 [POLICY_THREAD_SPEC.md](POLICY_THREAD_SPEC.md), 이행 계획은 https://claude.ai/code/artifact/a3599c22-7b09-4241-ade5-38336bf73672
>
> 각 항목은 **무엇을 · 왜**로 적는다. 되돌릴 때 필요한 정보를 함께 남긴다.

---

## 목차

| 단계 | 내용 | 날짜 | 상태 |
|---|---|---|---|
| [0](#단계-0--기반-정리) | 기반 정리 (게인 60/1, JointID, Makefile) | 2026-08-31 | 완료 |
| [1](#단계-1--mit-전면-전환) | MIT 전면 전환 | 2026-08-31 | 코드 완료 · 실기 미검증 |
| [2](#단계-2--중력-보상-분기) | 중력 보상 분기 | 2026-08-31 | 완료 |
| [3](#단계-3--정책-없는-배관) | 정책 없는 배관 | 2026-08-31 | 코드 완료 · 회귀 미검증 |
| [4](#단계-4--규약-이송과-골든-테스트) | 규약 이송 + 골든 테스트 | 2026-08-31 | 4-1 · 4-3 완료 |

---

## 단계 0 — 기반 정리

### 0-1 · 손목 게인을 학습 쪽 값으로 (`60 / 1`)

**파일** `config/motors.json`

| 모터 | 항목 | 변경 전 | 변경 후 |
|---|---|---|---|
| id 7 `right_wrist` | `control_kp` / `control_kd` | 40.0 / 4.0 | **60.0 / 1.0** |
| id 8 `left_wrist` | `control_kp` / `control_kd` | 40.0 / 4.0 | **60.0 / 1.0** |

페달(id 9, 10 = 50.0 / 1.0)은 건드리지 않았다.

**왜** — 학습 액추에이터가 손목을 `stiffness=2.1, damping=0.035`로 두고 있고, 주석이 그 값의 출처를
`실기 CST: kp(60) × gear(35)`로 명시한다. 실기는 `40 / 4`(관절 환산 1.4 / 0.14)여서 어긋나 있었다.
학습 쪽 주석에 `motors.json이 08-18에 60/1→40/4로 변경됨, sim 미반영`이라고 기록된 기지의 TODO였다.
재학습 대신 실기를 옮기고 결과를 관찰하기로 했다.

**영향** — 손목은 이미 연주 중 CST로 돌고 있으므로(`get_modes(true)`) **정책과 무관하게 지금의 연주가 바뀐다.**
감쇠비 ζ가 `(1/4) ÷ √(60/40)` = **1/4.9로 감소**, 고유진동수는 `√(60/40)` = **22% 증가**.
더 뻣뻣하고 덜 눌린 손목이 된다 — 타격 스냅에 유리, 임팩트 후 채터링에 불리.

**되돌리기** — `config/motors.json.bak-40-4`. 되돌릴 경우 반대 방향(학습을 1.4 / 0.14로 옮기고 fine-tune)을 택한다.

### 0-2 · `JointID`를 헤더로 이동

**파일** `include/common/robot_config.hpp` (추가) · `src/trajectory/behavior_planner.cpp:4-16` (삭제)

`JointID` 네임스페이스가 `.cpp`에 정의돼 있어 `Controller`에서 접근할 수 없었다.
함정 2의 머지 지점(`q[HEAD_YAW] -= pt.q[0]`)에 필요하므로 헤더로 옮겼다.
정책 담당 관절 개수 `JointID::NUM_ARM = 9`를 함께 추가했다.

### 0-3 · Makefile 오브젝트 디렉터리

**파일** `Makefile:33`

`directories` 타깃이 obj 하위 폴더를 하드코딩하고 있어 `src/policy/`를 추가하면 빌드가 깨진다.
`$(OBJDIR)/policy`를 미리 넣었다. (`SOURCES`는 `find`로 자동 탐색하므로 `.cpp`는 그냥 잡힌다.)

---

## 단계 1 — MIT 전면 전환

> 팔 TMotor 7축(id 0~6)을 **상시** MIT 모드로 구동한다.
> 명세는 연주 구간 진입·복귀 시 전환하도록 했으나, **모터 모드와 궤적 소유권은 별개**라
> 전환 기계를 없애고 비연주 동작(진입 4초 · POSE · IDLE)까지 학습과 같은 PD 법칙으로 통일했다.
>
> MIT 식 `τ = Kp(p_des − p) + Kd(v_des − qd) + t_ff` 가 학습 환경의 ImplicitActuator PD와 글자 그대로 같다.

### ⚠️ 명세가 다루지 않은 것 — MIT는 양방향 프로토콜이 다르다

| | 서보 모드 | MIT 모드 |
|---|---|---|
| 송신 CAN id | `node_id \| (packet<<8) \| CAN_EFF_FLAG` — 확장 프레임 | `canId & CAN_SFF_MASK` — **표준 프레임** |
| 수신 CAN id | `node_id` (모터별) | **0x00** — 식별자는 `data[0]` |
| 수신 레이아웃 | pos 2B(0.1°) · spd 2B · cur 2B · temp · err | pos 16b · vel 12b · torque 12b 팩 |
| 세 번째 값 | **전류** [A] | **토크** [N·m] |

수신 경로를 그대로 두면 허리(node 0)만 모든 프레임을 잘못 파싱하고 나머지 6축은 피드백이 끊긴다.
`first_recv_done` · 안전 검사 · (이후) 정책 obs가 함께 무너진다. 그래서 1-4가 필수다.

### 1-1 · 코덱 정리 — 미초기화 멤버 제거

**파일** `include/hardware/motor_codec.hpp` · `src/hardware/motor_codec.cpp`

`TMotorMITCodec`은 완성돼 있었지만 한 번도 인스턴스화되지 않은 죽은 코드였고,
**대입되는 곳이 없는 멤버 6개**를 clamp와 12비트 인코딩에 쓰고 있었다.

```
GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX, GLOBAL_I_MAX, Kt   ← 전부 미초기화
```

그대로 쓰면 `v_des`·`t_ff`가 쓰레기 값으로 잘리고 프레임이 깨진다.
게다가 AK10-9(허리)와 AK70-10(팔)이 다르므로 **클래스 전역이면 안 된다.**

**변경**

- `struct MotorMitLimits` 신설 — `p/v/t` 인코딩 범위, `kp/kd` 게인 범위, `kt`
- `GLOBAL_*` 멤버 **전부 제거** (`p/kp/kd` 기본값은 구조체 기본값으로 이동)
- 시그니처 변경 — 한계값을 모터별로 주입

```cpp
void encodeCommand(can_frame*, int canId, int dlc,
                   float p_des, float v_des, float kp, float kd, float t_ff,
                   const MotorMitLimits &lim);

std::tuple<int,float,float,float> decodeFeedback(can_frame*, const MotorMitLimits &lim);
```

`decodeFeedback`은 세 번째 값을 `GLOBAL_I_MAX`(전류)가 아니라 `lim.t_min/t_max`(토크)로 디코딩하도록 고쳤다 —
MIT 프로토콜이 주는 것은 토크다.

### 1-2 · 설정값을 `motors.json`으로

**파일** `include/hardware/motor.hpp` · `src/hardware/motor.cpp` · `src/hardware/robot.cpp` · `config/motors.json`

`TMotor`에 필드 6개와 `mit_limits()` 헬퍼를 추가하고 JSON 파서에 연결했다.
`current_motor_current`의 의미가 모드별로 달라지므로 원본 토크를 담을 `current_torque_mit`도 추가했다.

**설정값**

| 모터 | 모델 | `mit_kp` | `mit_kd` | `mit_p_limit` | `mit_v_limit` | `mit_t_limit` | `mit_kt` |
|---|---|---|---|---|---|---|---|
| 0 waist | AK10_9 | 100.0 | 5.0 | 12.5 | 50.0 | **65.0** | **1.611** |
| 1~6 팔 | AK70_10 | 100.0 | 5.0 | 12.5 | 50.0 | **25.0** | **1.034** |

- `mit_kp` / `mit_kd` — 학습 액추에이터(`drumrobot_cfg.py:108-121`)와 **같은 값이어야 한다.**
  `Kd ≤ 5`는 CubeMars 매뉴얼 상한.
- `mit_kt` — 출력축 토크 → 전류 [N·m/A]. **학습 `effort_limit` ÷ 실기 `current_limit`에서 유도**했다.
  AK10_9 `48.0 / 29.8 = 1.611`, AK70_10 `24.0 / 23.2 = 1.034`.
  (학습 cfg가 "current_limit 23.2A와 정합"이라고 적어 둔 쌍이라 두 레포에 이미 있는 숫자로 닫힌다.)

> **⚠️ 검증 필요** — `mit_v_limit`, `mit_t_limit`은 MIT 프로토콜의 **인코딩 범위**(모터 펌웨어 상수)이지
> 물리 한계가 아니다. 데이터시트 근사값이므로 매뉴얼과 대조해야 한다. 틀리면 12비트 인코딩에서
> **조용히 잘린다.** 재컴파일 없이 고칠 수 있도록 JSON에 두었다. 1-7에서 지령↔실측을 대조할 것.

**최상위 스위치** — `config/motors.json`에 `"tmotor_mit": true`를 추가했다.
`false`로 두면 송신·수신 양쪽이 서보 모드로 돌아간다.

### 1-3 · 송신 분기

**파일** `include/common/control_queue.hpp` · `include/common/app_context.hpp` ·
`include/realtime/controller.hpp` · `src/realtime/controller.cpp` ·
`include/trajectory/trajectory_generator.hpp` · `src/trajectory/trajectory_generator.cpp` · `src/main.cpp`

- `ControlMode::MIT` 추가
- `AppContext`에 `tmotor_mit` / `gain_ramp` / `gain_ramp_target` / `mit_enter_requested` / `policy_active` 추가
- `main.cpp` — `robot.initialize()` 직후 `ctx.tmotor_mit = robot.use_mit`
- `TrajectoryGenerator::initialize()` — `tmotor_control_mode`를 `ctx.tmotor_mit`에 따라 `MIT` 또는 `VEL`로 설정
- `Controller`에 `TMotorMITCodec mit_codec` 멤버 추가
- `tmotor_send_task()`에 `ControlMode::MIT` 분기

```cpp
if (mode == ControlMode::MIT) {
    double ramp  = ctx.gain_ramp.load();
    double p_des = (ramp < 1.0) ? tmotor->current_position : motor_position;

    mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
        p_des,
        0.0f,                       // v_des = 0  ← 학습의 Kd는 순수 감쇠항 (함정 3)
        tmotor->mit_kp * ramp,
        tmotor->mit_kd * ramp,
        0.0f,                       // t_ff = 0
        tmotor->mit_limits());
}
```

**`v_des = 0`인 이유** — 학습 환경은 `set_joint_position_target`만 호출하고 속도 타겟은 기본값 0이다.
`τ = Kp(q_des−q) + Kd(0−qd)`에서 Kd는 **순수 감쇠**다. 정책 속도를 넣으면 감쇠항이 추종항으로 바뀐다.

**`p_des`를 램프 중 실측으로 고정하는 이유** — 목표 자세를 물린 채 게인을 올리면 그 오차만큼 즉시 토크가 터진다.
실측을 넣으면 위치 오차가 0이라 토크도 0에서 출발하고, 궤적이 실제로 움직이기 시작할 때만 토크가 생긴다.

기존 안전 검사(급변 차단 `POS_DIFF_LIMIT`, `min/max_angle`)는 분기 **앞에** 그대로 두어 MIT에도 걸린다.
로그는 mode 열에 `3.0`(MIT)으로 기록되고 마지막 열이 `ramp`다.

### 1-4 · 수신 분기

**파일** `src/realtime/controller.cpp` — `distribute_frames()`

```cpp
const bool mit = ctx.tmotor_mit.load();

// 매칭 규칙 자체가 다르다
const bool match = mit ? (frame.data[0] == tmotor->node_id)
                       : ((frame.can_id & 0xFF) == tmotor->node_id);

if (mit) {
    auto [mid, pos, spd, torque] = mit_codec.decodeFeedback(&frame, tmotor->mit_limits());
    ...
    tmotor->current_torque_mit    = torque;
    tmotor->current_motor_current = torque / tmotor->mit_kt;   // 과전류 검사 의미 유지
}
```

**전류 환산의 이유** — `safety_check_recv_tmotor()`가 `current_motor_current > current_limit`을
연속 5회로 판정한다. MIT는 토크를 주므로 `kt`로 환산해야 `motors.json`의 `current_limit` 임계값이
지금과 같은 의미를 유지한다. 이 검사 코드 자체는 손대지 않았다.

### 1-5 · 영점

**파일** `src/hardware/robot.cpp` — `set_zero_tmotor()`

MIT 영점(`0xFE`)은 제어 모드 안에서만 받는다. **enter → setzero → exit** 순서로 감쌌다.
enter 자체는 통전시키지 않는다 — 첫 command 프레임에서 kp/kd가 실려야 토크가 생긴다.

런타임에 서보 전용 명령을 쓰던 곳은 여기 하나뿐이었다
(`encodeCurrentBrake` / `encodePositionVelocity`는 호출자 없는 죽은 코드).

### 1-6 · `START` 토크 인가

**파일** `src/trajectory/behavior_planner.cpp` — `handle_start()` · `src/realtime/controller.cpp`

```cpp
if (ctx.tmotor_mit.load()) {
    ctx.gain_ramp = 0.0;
    ctx.mit_enter_requested = true;    // send_thread가 처리
    ctx.gain_ramp_target = 1.0;
}
```

`Controller`에 두 함수를 추가했다.

- `enter_mit_control_mode()` — id 0~6에 제어 모드 진입 프레임 송신.
  **CAN 쓰기는 send_thread가 소유**하므로 요청 플래그를 거쳐 `send_loop`의 `cnt==0`에서 실행된다.
  (recv_thread가 CAN에 쓰는 기존 경로는 Maxon shutdown 하나뿐이고, 그걸 늘리지 않으려는 의도.)
- `advance_gain_ramp()` — 5ms마다 `ROBOT::DT_SECOND / 0.5` = 0.01씩 target으로 이동. 0→1에 0.5초.

### 1-8 · 초기화 교착 수정 (1-3/1-4 재검토 중 발견)

**파일** `include/hardware/robot.hpp` · `src/hardware/robot.cpp` · `include/realtime/controller.hpp` · `src/realtime/controller.cpp`

**증상** — `START`를 눌러도 `send_loop`이 `all_tmotors_received()` 게이트에서 영원히 멈춘다.

**원인** — 서보 모드에서는 모터가 상태 프레임을 스스로 뿌리지만, **MIT는 command 프레임에 대한
응답으로만 피드백을 준다.** `send_loop`은 송신 전에 첫 수신을 기다리고, 모터는 명령을 기다린다.
게다가 1-5의 `set_zero_tmotor()`가 마지막에 `exit`을 보내 제어 모드 밖으로 나온 상태였다.

**수정 둘**

1. `Robot::enter_mit_mode()` 신설 — `initialize()`의 `set_zero_tmotor()` 직후에 호출해
   TMotor를 MIT 제어 모드에 넣어 둔다. **진입 자체는 통전시키지 않는다** (kp/kd가 실린 command를
   받아야 토크가 생긴다). 즉 `Standby`의 "토크 미인가 대기"가 그대로 유지된다.
2. `Controller::prime_mit_feedback()` 신설 — 게이트를 통과하지 못한 동안 `kp = kd = t_ff = 0`
   command를 보낸다. **토크는 0이고 피드백 프레임만 돌아온다.**

게인이 0이므로 이 두 변경 어느 쪽도 모터를 움직이지 않는다.

### 되돌리기

`config/motors.json`의 `"tmotor_mit"`을 `false`로. 서보 인코더·디코더 함수와 `control_gain`은
**지우지 않고 남겨 두었다.** 수신 분기가 모터별 런타임 플래그가 아니라 이 전역 설정 하나로 갈리므로,
이중 경로의 복잡도 없이 설정 한 줄로 서보 복귀가 된다.

### 검증 상태

- [x] 클린 빌드 · 경고 0
- [ ] **1-7 실기 검증** — 고정 키 꽂고 `START` → 자세 유지 확인 → 키 제거 →
      `POSE`/`MOVE`로 처짐 관찰 → 악보 연주

> **예상되는 변화** — 서보 VEL의 `control_gain`(1000~4000, ERPM 단위 P항)은 뻣뻣하게 자세를 붙들지만
> MIT `Kp=100`은 훨씬 컴플라이언트하고 TMotor에는 중력 보상이 없다(`cal_torque()`는 Maxon 전용).
> **정지 자세가 중력만큼 처진다.** 정책에겐 sim에서 겪던 그대로라 정상이지만 `POSE|ready`가
> 눈에 띄게 언더슈트할 수 있다. 링크 질량이 코드에 없어 크기는 계산 불가 — 실기에서 봐야 한다.
> 거슬리면 **비연주 구간만 `mit_kp` 상향**으로 대응한다 (모드가 아니라 숫자만 바꾸므로 전환 위험 없음).

---

## 단계 2 — 중력 보상 분기

**파일** `src/realtime/controller.cpp` — `cal_torque()`

```cpp
if (ctx.policy_active.load()) {
    return torque_mNm;      // 중력 보상 항을 빼기 전에 반환
}
```

**왜** — 학습 제어 법칙에는 보상항이 없다. 그런데 **시뮬 물리에는 중력이 있으므로**
시뮬 안의 팔은 `q_des`보다 살짝 아래에 머문다(정상상태 처짐). 정책은 수백만 스텝 동안 그 처짐을 겪으며
**처짐을 감안한 `q_des`를 내도록** 학습됐다 — 원하는 위치보다 조금 위를 지시하는 습관이 정책 안에 있다.

실기에서 보상을 켜면 그 처짐이 사라지므로 스틱이 실제로 조금 위에 뜬다. **이중 보상**이다.
보상을 끄는 것은 정확도를 포기하는 게 아니라 정책이 학습한 세계를 실기에 재현하는 일이다.

`policy_active`가 아직 켜지는 곳이 없으므로 **현재 동작 변화는 없다.** 단계 5의 전제만 놓은 것.

### 관련 — 손목은 이미 CST다 (명세 정정)

명세 함정 5는 "실기는 `wrist_control_mode = CSP`로 연주한다"고 적었으나 **코드와 다르다.**

```cpp
// src/trajectory/trajectory_generator.cpp:286
if (is_play) wrist_control_mode = ControlMode::CST;
else         wrist_control_mode = ControlMode::CSP;
```

`generate_play_trajectory()`가 `get_modes(true)`로 호출한다(224행). 헤더의 멤버 기본값이
`= ControlMode::CSP`라 선언부만 보면 CSP로 보이는 것이 원인으로 보인다.

따라서 **제어 모드는 변경할 것이 없고**, 결정 01의 실제 변경분은 위의 중력 보상 분기 하나다.
그리고 `motors.json`의 `control_kp/kd`는 **지금 실연주에 쓰이고 있는 값**이므로 0-1의 변경이
정책과 무관하게 즉시 관찰 가능하다.

---

## 단계 3 — 정책 없는 배관

> 정책이 붙을 자리를 전부 만들되 정책은 넣지 않는다. `policy_active`가 `false`인 동안
> **기존 경로가 그대로 동작**하므로 회귀 기준선이 유지된다 (열린 결정 → `policy_active` 분기 채택).

### 3-1 · 단일 클록

**파일** `include/common/control_queue.hpp` · `include/common/app_context.hpp` · `src/realtime/controller.cpp`

- `ControlSetPoint::t_score` — 이 setpoint의 악보 시간 [s]
- `AppContext::tick` — `send_loop`이 5ms pop마다 `fetch_add(1)`. **유일한 시간 권위**
- `AppContext::t_score` — pop한 setpoint의 `t_score`를 발행
- `AppContext::policy_fault`, `policy_mtx`, `policy_cv`

**왜 `t_score`를 setpoint에 싣나** — `get_num_point()`가 `round_sum` 누산기로 반올림 오차를 흡수하고
`play_speed_scale`까지 반영한다. 정책이 `tick × 5ms`로 시간을 따로 계산하면 **배속 재생에서 팔과 발이 어긋난다.**
시간의 소유자를 문자 그대로 하나로 만든다.

### 3-2 · 소유권 가드

**파일** `include/trajectory/trajectory_generator.hpp` · `src/trajectory/trajectory_generator.cpp`

`control_queue.push()` 직접 호출 **7군데를 전부** `push_setpoint()`로 교체했다
(`standby` / `joint_space` / `task_space` / `play_start` / `play_end` / `play` / `idle`).
한 군데라도 빠지면 가드가 새는 지점이 된다.

```cpp
void TrajectoryGenerator::push_setpoint(ControlSetPoint& sp) {
    if (ctx.policy_active.load()) {
        for (int j = 0; j < JointID::NUM_ARM; ++j) {
            sp.q[j] = 0.0; sp.qd[j] = 0.0;
            sp.mode[j] = ControlMode::NONE;   // "이 관절은 내 소유가 아님"
        }
    }
    sp.t_score = cur_t_score;
    control_queue.push(sp);
}
```

**NONE이 곧 fail-safe다** — `tmotor_send_task`/`maxon_motor_send_task`가 `NONE`을 만나면 프레임을
보내지 않고, **프레임 미송신은 모터가 직전 명령을 유지**한다는 뜻이다. 머지가 어떤 이유로든 실패해도
팔은 자동으로 홀드된다.

### 3-3 · 연속성

**파일** `src/trajectory/trajectory_generator.cpp`

`sync_last_q_from_robot(const Robot&)` 신설 — 실측 팔 관절각을 `last_q[0..8]`에 되쓴다.
`9~12`(발·머리)는 planner가 계속 소유하므로 건드리지 않는다.

**왜** — PLAYING 동안 팔은 정책이 움직이는데 `TrajectoryGenerator`는 그걸 모른다.
그냥 두면 END 시점의 `last_q`가 **START 때 멈춘 낡은 값**이라 복귀 궤적 첫 setpoint가
실제 자세에서 수십 도 떨어진 곳으로 점프한다 (함정 1).

> 아직 호출하는 곳이 없다. 단계 5에서 `policy_active`를 내리기 **직전에** 부른다 (순서가 중요).

### 3-4 · `play_motion_generator` — 팔 생성 자체를 끈다

**파일** `src/trajectory/play_motion_generator.cpp` — `generate_motion()`

정책 구간에서는 `base_motion_generator` / `state_motion_generator`를 **호출하지 않는다.**

```cpp
const bool policy_owns_arm = ctx.policy_active.load();

// 발·머리 — 정책이 소유하지 않으므로 항상 생성
head_motion  = head_motion_generator.generate_motion(rds, n);
pedal_motion = pedal_motion_generator.generate_motion(rds, n, dt);

// 팔·허리(0~8) — 정책이 소유하면 아예 생성하지 않는다
if (!policy_owns_arm) {
    base_motion  = base_motion_generator.generate_motion(rds, n, dt);
    state_motion = state_motion_generator.generate_motion(rds, n, dt);
    if (base_...get_error() || state_...get_error()) return empty;   // 이 게이트도 함께 비활성
}
```

**값만 버리는 것으로는 부족하다** — 두 생성기는 호출마다 내부 상태를 갱신하고
(`state`의 4상태 머신, `base`의 error 플래그), `get_error()`가 서면 **정책과 무관한 IK 실패가
구간 폐기 → `play_abort`로 이어진다.** 명세 00절의 "PLAYING 호출과 `solve_ik`를 제거한다"가 이 뜻이다.

> 허리(id 0)도 정책 소유다. 학습 `ctrl_joint_names`가 `waist_joint`를 포함한 9관절이므로
> `base`가 만들던 `b.waist`도 함께 사라진다.

**함께 고친 것**

- `std::array<double,13> q;` → `q{}` **zero-init** (함정 6). 팔 계산을 건너뛰면 `q[0..8]`이
  쓰레기 값으로 남는다. 0 역시 팔에겐 위험한 자세이므로 **값이 아니라 모드로 막는 것이 요점**이다.
- `q[11] = h.yaw - q[0]` → 정책 구간에서는 **raw `h.yaw`**. 허리 보정은 머지 지점에서 한다 (함정 2).
  여기엔 쓸 `q[0]`이 없기 때문이다. 비정책 경로는 기존대로 여기서 보정한다.

`PlayMotionGenerator::reset()`(DRUM/START 진입 자세)은 **건드리지 않았다** — planner 소유 구간이다.

### 3-5 · 상태 스냅샷

**파일** `include/common/joint_snapshot.hpp` (신규) · `include/hardware/robot.hpp` · `src/realtime/controller.cpp`

`JointSnapshot` (seqlock) 신설. `distribute_frames()` 끝에서 팔 9관절을 **한 번에** 발행한다.

**왜** — `recv_loop`은 100µs마다 돌며 프레임이 도착한 모터부터 순서 없이 갱신한다.
정책이 모터를 하나씩 순회해 읽으면 **"관절 0은 t, 관절 5는 t+50µs"인 뒤섞인 상태**가 obs로 들어간다 (함정 4).

- 프레임 갱신이 하나도 없었으면 발행하지 않는다 (recv는 100µs마다 돌지만 프레임은 그보다 드물다)
- 손목(7,8)은 Maxon이라 속도 피드백이 없어 `qd`를 0으로 둔다.
  스냅샷에 `t_pub_ns`를 실어, 단계 5에서 `PolicyRunner`가 자기 주기(`POLICY_DT` 15ms)로 유한차분한다 —
  **100µs 간격 차분은 엔코더 양자화 노이즈가 커서 쓸 수 없다.**

### 3-6 · 머지와 워치독

**파일** `include/policy/policy_target.hpp` (신규) · `include/realtime/controller.hpp` · `src/realtime/controller.cpp`

`PolicyTarget` (seqlock 슬롯) 신설 — `q[9] + tick + valid`. **큐가 아니다.**
정책을 `ControlQueue`에 넣으면 100ms 버퍼 뒤에 줄을 서게 되어 폐루프가 성립하지 않는다.

`send_loop`의 `cnt==0` 블록에 추가:

```cpp
uint64_t t = ctx.tick.fetch_add(1) + 1;
if (ctx.policy_active.load()) {
    merge_policy_target(t);
    if (t % POLICY_TICK_STRIDE == 0) ctx.policy_cv.notify_one();   // 3틱 = 15ms = 66.7Hz
}
```

`merge_policy_target()`:
- 슬롯이 유효하고 `WATCHDOG_TICKS`(6틱 = 30ms) 이내면 팔 `q[0..8]`을 덮어쓰고
  모드를 `MIT`(0~6) / `CST`(7,8)로 세운다. `qd`는 0 (MIT `v_des = 0`)
- 낡았으면 **아무것도 하지 않는다** — planner가 넣은 `NONE`이 남아 프레임이 나가지 않고 모터가 홀드.
  `policy_fault`와 `play_abort`를 세운다
- 머리 yaw 허리 보정 `q[HEAD_YAW] -= pt.q[WAIST]` (3-4의 raw 규약과 짝)

### 3-7 · 배선

**파일** `src/main.cpp` · `include/realtime/controller.hpp`

`PolicyTarget`을 만들어 `Controller` 생성자에 주입했다.
**`policy_thread`는 아직 만들지 않았다** — `PolicyRunner`가 없어 빈 스레드를 띄울 이유가 없다.
단계 5에서 우선순위 25로 추가한다 (recv 30보다 낮게, planner 20보다 높게).

### 발견해 고친 것 — seqlock 재시도 버그

`do { ... if (s0 & 1u) continue; ... } while (s0 != s1);` 에서 **`continue`는 루프 처음이 아니라
조건식으로 점프**한다. 쓰기 중(홀수 seq)을 만나면 초기화되지 않은 `s1`로 조건을 평가했다.
`for(;;)` + 명시적 `break`로 두 seqlock을 모두 고쳤다. (컴파일러 경고로 발견)

### 검증 상태

- [x] 클린 빌드 · 경고 0
- [ ] **회귀 확인** — `policy_active = false` 상태에서 기존 연주가 그대로 되는지.
      배관이 기존 동작을 건드리지 않았다는 증명이다.

---

## 단계 4 — 규약 이송과 골든 테스트

> 추론 런타임은 **ONNX Runtime**, 정규화는 **전부 그래프 안**으로 확정.
> 근거와 전체 명세: https://claude.ai/code/artifact/d3d6b309-3e48-4ca5-93bb-ebdd3c2c0c07

### 4-1 · `export_policy.py` — 완료

**파일** `<학습레포>/drum_robot/scripts/deploy/export_policy.py` (신규, 학습 레포 기존 코드 무수정)

```
policy.onnx         env 정규화 + 스케일러 + MLP + 결정적 평균 (401 KB, opset 13)
obs_constants.json  obs_builder 가 쓸 상수 + 관절 매핑
```

**대상 런** `2026-08-31_09-55-09` (500k 스텝 완료). 손목 게인 2.1/0.035 = 실기 kp60/kd1 —
`motors.json` 과 일치. 진행 중이던 `13-52-32` 런은 실험용으로 손목이 1.4/0.14 라 쓰지 않는다.

**검증** PyTorch vs onnxruntime 최대 오차 **1.01e-06** (32회 랜덤 입력, 허용 1e-5).

**Isaac 없이 돈다** — 체크포인트가 자기 서술적이다(`net_container.{0,2,4,6}` 이 Linear,
홀수가 ELU)이므로 skrl 도 IsaacLab 도 필요 없다.

**그래프 입력 규약** — 이름 있는 원시 버퍼 9개. 위치는 전부 **허리 기준 미터**.

| 입력 | 형상 | | 입력 | 형상 |
|---|---|---|---|---|
| joint_pos | [1,9] | | hit_armed | [1,2,8] |
| joint_vel | [1,9] | | arm_role | [1,2] |
| tip_pos | [1,2,3] | | per_arm_pos | [1,2,2,3] |
| drum_pos | [1,8,3] | | per_arm_time | [1,2,2] |
| next_hits | [1,6,11] | | **출력** action_mean | [1,9] |

#### z 문제 해결 — `DRUM_CENTER_EFF`

sim 은 런타임 `drum_pos`/`tip_pos` 를 월드 프레임(`+robot_waist_joint_offset_z` = 1.0)에서
정규화하는데, 정규화 상수는 오프셋 없는 배열에서 만들어졌다 (`env.py:817` vs `748`).
그 결과 z 14채널이 clamp(±1.5)에 상시 포화해 **스케일러가 상수로 학습**했다.

체크포인트 측정으로 확인: 해당 채널 mean **1.5000**, std **8.909e-05** — 148채널 중 최소 상위를 독점.

```python
DRUM_CENTER_EFF = drum_center - [0, 0, robot_waist_joint_offset_z]
```

이 상수 한 줄로 C++ 이 허리 기준 z 를 그대로 넣어도 sim 과 같은 값이 나온다.
**"1.5 하드코딩"이 필요 없다.** 오프셋은 런 파라미터에서 읽으므로, 나중에 sim 을 고쳐도
재export 만 하면 C++ 은 손댈 것이 없다.

스크립트가 포화 채널을 스스로 찾아 출력한다 — 예측한 14개와 정확히 일치했다.

#### 관절 순서 — 세 후보가 전부 다르다

`robot_interface.py:27` 이 `robot.find_joints()` 를 `preserve_order` 없이 호출하므로
**USD articulation 순서**가 이긴다. `specs.py` 의 `ctrl_joint_names` 목록 순서가 아니다.

| | obs 인덱스 → 실기 모터 id |
|---|---|
| `specs.py` 목록 순서 | 0, 2, 5, 6, 1, 3, 4, 8, 7 |
| URDF 선언 순서 | 0, 2, 5, 6, 8, 1, 3, 4, 7 |
| **articulation (채택)** | **0, 2, 1, 5, 3, 6, 4, 8, 7** |

근거: `body_names` 출력이 링크 트리의 **너비우선 열거와 정확히 일치**했다
(`base_link, waist, head, left_shoulder_1, right_shoulder_1, head_2, ...`).
관절은 루트를 제외한 링크와 1:1 이고 자식 링크 이름을 갖는다.

> **아직 추정이다.** 4-2(`dump_golden.py`)가 실제 `ctrl_joint_names` 를 내보내고
> 골든 비교가 불일치를 기계적으로 잡는다. 사람이 기억할 필요가 없다.

#### 드럼 좌표 변환

실기는 악기마다 `right`/`left` 좌표를 따로 갖는데 학습은 하나만 쓴다. 8개 전수 대조 결과:

```
학습 좌표 = midpoint(real.right, real.left) - (0, 0, 0.045)
```

XY 오차 0.5mm 이내(반올림), Z 는 8개 전부 정확히 +45mm.
z 오차는 clamp 에 흡수되지만 **XY 중점 변환은 흡수되지 않는다** — 빼먹으면 최대 26mm 어긋난 곳을 조준한다.

#### 넣은 안전장치

작업 중 실제로 사고가 났고(진행 중인 런을 export, 게인이 반대로 어긋남) 그래서 넣었다.

1. **런 파라미터 기반** — 상수를 라이브 소스가 아니라 런이 시작할 때 덤프한
   `params/env.yaml` 에서 읽는다. 학습 후 소스가 바뀌면 조용히 어긋나는 것을 막는다.
2. **소스 드리프트 경고** — 런 시작 이후 수정된 파일을 찍는다. 실제로 `drumrobot_cfg.py` 를 잡았다.
3. **`--motors-json` 교차검증** — 손목 CST 게인이 학습과 안 맞으면 **export 를 중단**한다 (exit 2).
4. **`--checkpoint` 명시 필수** — mtime 자동 선택은 학습 중에 진행 중인 런을 고른다.

#### 고친 것

- `policy_dt` 0.01 → **0.015**. 실기 3틱(66.7Hz)이 정답이고, 스텝→초 변환은
  학습 스텝 주기(`sim_dt × decimation` = 1/60s)로 해야 한다.
  그 결과 `hit_window_s = 0.05`, `rearm_s = 0.1167` — 명세 04절 값과 일치.

### 4-3 · ONNX Runtime vendoring — 완료

**파일** `lib/onnxruntime/fetch.sh` (신규) · `Makefile` · `tools/ort_check.cpp` (신규) · `.gitignore`

- **fetch.sh** — v1.23.2 CPU 빌드, sha256 고정. 바이너리는 커밋하지 않는다(22MB).
  로봇 머신에서 최초 1회만 네트워크 필요.
- **Makefile** — `ORT_DIR`, `INCLUDE`, `LDFLAGS` + `-Wl,-rpath,'$ORIGIN/../lib/onnxruntime/lib'`.
  `$ORIGIN` 상대 rpath 라 **시스템 설치가 필요 없다.**
- **`make ort-check`** — vendoring 과 export 를 한 번에 확인하는 타깃.
  `main.out` 에는 포함되지 않는다(`SOURCES` 는 `src/` 만 훑음).

**결과** — 입력 9개·출력 1개가 명세와 정확히 일치. 그리고 실시간 우려가 측정으로 끝났다:

| 추론 시간 (2000회, 워밍업 50회 후) | |
|---|---|
| 평균 | 13.9 µs |
| p50 | 13.1 µs |
| **p99** | **22.5 µs** |
| 최대 | 40.9 µs |
| 예산 | 15,000 µs (정책 주기) · 워치독 30,000 µs |

**p99 가 예산의 0.15%.** 명세 05절이 걱정한 우선순위 역전이 터져도 여유가 압도적이다.
세션은 `SetIntraOpNumThreads(1)` · `ORT_SEQUENTIAL` · 입출력 텐서 사전할당으로 설정했다.

> `main.out` 은 아직 ORT 심볼을 쓰지 않아 링커가 떨어낸다(`ldd` 에 안 보임). 정상이다 —
> 단계 5에서 `policy_runner` 가 들어오면 링크된다. 링크·rpath 자체는 `ort-check` 가 증명했다.

### 4-4 · `normalizer.cpp` — 만들지 않는다

정규화가 전부 그래프 안에 있다. C++ 이 다루는 정규화는 없다.

### 배포 산출물

`data/policy/policy.onnx` · `data/policy/obs_constants.json` (추적 대상 — 로봇에 올라간
정책이 어느 런에서 나왔는지 기록으로 남기기 위해).

---

## 아직 하지 않은 것

| 단계 | 내용 | 비고 |
|---|---|---|
| 4 | 나머지 — 4-2 `dump_golden.py`(Isaac 필요), 4-5 `policy_score.hpp`, 4-6 `obs_builder.cpp`, 4-7 골든 비교 | |
| 5 | 정책 가동 | |
| 6 | 실기 브링업 | 사용자 주도 |

### 학습 레포 (미변경)

`/home/shy/RL_workspace/IsaacLab/source/extensions/drum_robot` — **아직 아무것도 고치지 않았다.**
0-1에서 실기를 학습 값에 맞췄으므로 손목 게인 수정과 fine-tune은 필요 없어졌다.

---

## 2026-09-01 — 정책 스레드 완성 + 주기 100Hz 전환

### 배경: 주기를 66.7Hz -> 100Hz 로 바꾼 이유

정책 주기는 학습 주기와 같아야 한다. 액션이 각속도이고 한 스텝 동안 적분되므로
(`drumrobot_env.py:374`), 주기가 폐루프 거동에 박혀 있다.

이전: 학습 60Hz (`SIM_DT 1/120 x decimation 2`) -> 실기 5ms 격자에 반올림 -> 3틱 = 66.7Hz (+11% 이탈)
현재: 학습을 100Hz 로 재학습 (`SIM_DT 0.005 x decimation 2`) -> 실기 2틱 = 100Hz (이탈 0)

100Hz 를 고른 근거는 "200/2 가 딱 떨어져서" 가 아니다 (15ms 도 정확히 3틱이다).
물리 스텝이 실기 send 틱과 같아지고, ZOH 실효 지연이 7.5ms -> 5ms 로 줄고,
타격 시점 양자화가 15ms -> 10ms 로 좋아지기 때문이다.

### 변경

| 파일 | 내용 |
|---|---|
| `include/common/robot_config.hpp` | `POLICY_TICK_STRIDE = 2`, `POLICY_DT_SECOND` 신설. 주기의 유일한 권위 |
| `include/realtime/controller.hpp` | 로컬 STRIDE 상수 제거, 공용 상수 참조. `WATCHDOG_TICKS = 3 * STRIDE` |
| `include/common/control_queue.hpp` | `ControlSetPoint::policy_owns_arm` 추가 |
| `include/common/app_context.hpp` | `policy_ready`, `policy_epoch` 추가 |
| `src/realtime/controller.cpp` | 깨우기는 `ctx.policy_active`, 머지는 `curr_point.policy_owns_arm` 로 분리 |
| `src/trajectory/trajectory_generator.cpp` | `acquire_policy()` / `release_policy()` 구현. START 궤적 끝에 획득, END 궤적 앞에 반납 |
| `include/trajectory/trajectory_generator.hpp` | `Robot&` 주입, `sync_last_q_from_robot()` 인자 제거 |
| `src/trajectory/motion_planner.cpp` | TrajectoryGenerator 생성자에 robot 전달 |
| `include/policy/policy_runner.hpp` | 신규 |
| `src/policy/policy_runner.cpp` | 신규 — ORT 세션, cv 대기, obs 조립, 적분, 슬롯 발행 |
| `src/main.cpp` | PolicyRunner 생성 + prio 25 스레드. 초기화 실패 시 스레드를 띄우지 않음 |

### 해결한 함정

**소유권 경계 (신규 발견)** — `play_motion_generator` 와 `push_setpoint` 는 궤적 생성
시점(약 100ms 전)에 `ctx.policy_active` 를 읽고, `send_loop` 은 소비 시점에 읽었다.
경계에서 "생성 때 정책 / 소비 때 아님" 이 되면 팔에 `q=0` 이 그대로 나간다.
소유권을 `ControlSetPoint` 에 실어 보내 구조적으로 어긋날 수 없게 했다.

깨우기만 전역 플래그를 쓴다 — 첫 정책 소유 setpoint 보다 약 100ms 먼저 서므로
그 선행 구간이 슬롯을 채우고 ObsBuilder 의 팁·손목 이전값을 예열한다.

**함정 1 (`sync_last_q_from_robot` 미호출)** — `release_policy()` 안에서 호출.
END 궤적을 만들기 **전에** 부른다.

**함정 8 (`ObsBuilder::reset()` 미호출)** — `policy_epoch` 카운터.
`acquire_policy()` 가 +1 하고 PolicyRunner 가 변화를 보고 reset 한다.

**주기 불일치 가드 (신규)** — `PolicyRunner::initialize()` 가 `obs_constants.json` 의
`policy_tick_stride` / `policy_dt` 를 `ROBOT::` 상수와 대조해 어긋나면 거절한다.
`train_step_dt` 와 다르면 경고만 남긴다(과도기 검증용). 조용히 틀리는 종류의
오류라 반드시 막아야 했다.

### 정책을 켜지 않는 조건 (모두 개루프로 안전 강등)

- `PolicyConfig` 무효 / `obs_constants.json` 없음
- ORT 세션 생성 실패, 입출력 이름·개수 불일치
- 주기 불일치 (위 가드)
- 팔 모터 0~8 중 결번, 관절 한계 뒤집힘
- `ctx.tmotor_mit == false` (MIT 아니면 위치 목표를 실을 곳이 없다)

### 남은 일

- **재export 필요** — 현재 `data/policy/*` 는 STRIDE=3 시절 산출물이라 가드가 거절한다.
  학습 레포 100Hz 재학습 후 `export_policy.py` (`policy_tick_stride` 2 로) + `dump_golden.py`.
- 실기 검증 H1/H2 (MIT 확인, `motors` 로그로 클리핑·지연·처짐)
- `mit_v_limit 50` / `mit_t_limit 65·25` 는 미검증 추정값

---

## 2026-09-01 (2) — 주기를 66.7Hz 로 확정

### 경위

100Hz 로 재학습했으나 **성공률이 떨어져** 66.7Hz(3틱)로 되돌린다.

제어 루프 분석으로는 100Hz 가 불리할 이유가 없었다 — Δq_max 와 τ_max 가
`action_scale` 보정으로 보존되고(0.1667 rad / 16.7 N·m), 정상상태 각속도도
3.03 vs 2.90 rad/s 로 4.5% 차이뿐이다. PD 감쇠항(Kd=5)이 유지시간 기여분
(Kp·T/2 = 0.5~0.75)을 7~10배 압도하기 때문이다.

유력한 원인은 학습 하이퍼파라미터다. `rollouts` 가 64 로 고정된 채
`discount_factor` 만 올라가 **지평/rollout 이 1.56 → 2.60 으로 악화**됐다
(60Hz 1.56 / 66.7Hz 1.74 / 100Hz 2.60). rollout 이 커버하는 sim 시간이
1.067s → 0.640s 로 40% 줄어 GAE 의 부트스트랩 부담이 1.67배 늘었다.
다만 이는 가설이고 검증하지 않았다 — 실측이 우선이라 66.7 로 간다.

### 변경 (서버)

| 파일 | 내용 |
|---|---|
| `include/common/robot_config.hpp` | `POLICY_TICK_STRIDE` 2 → **3** |
| `include/policy/policy_config.hpp` | 기본값 `policy_dt` 0.015 / `train_step_dt` 0.015 / stride 3 |
| `src/policy/obs_builder.cpp` | 손목 유한차분 주석 10ms → 15ms |
| `tools/ort_check.cpp` | 예산 하드코딩(15000us) 제거 → `ROBOT::POLICY_DT_SECOND` 에서 산출 |

`POLICY_TICK_STRIDE` 하나가 주기의 권위다. Controller 의 notify 간격, 워치독,
PolicyRunner 의 정합 검사가 전부 여기서 나온다.

### 주의 — 기존 데이터 파일이 이제 통과한다

`data/policy/*` 는 60Hz 학습 + STRIDE=3 export 산출물이라, STRIDE 를 3 으로
되돌린 지금 **하드 가드를 통과한다** (100Hz 시절엔 거절됐다).

```
stride     json 3 == 실기 3        통과
policy_dt  json 0.015 == 0.015     통과
train_step_dt 0.016667 vs 0.015    경고만 (10% 차이)
```

즉 지금 서버를 띄우면 **60Hz 정책이 66.7Hz 로 실제로 가동된다.** 이는 원래
의도한 과도기 구성(+11% 이탈)이고, 새 학습이 끝나기 전에 배관을 실기로
점검해 볼 수 있다는 뜻이기도 하다. 재학습·재export 후에는 경고도 사라진다.

### 성능 (66.7Hz 기준 재측정)

```
추론  평균 12.9us  p50 12.6  p99 19.7  최대 42.7us
예산  15,000us  (사용률 0.28%)
```

### 학습 레포에서 해야 할 것 (사용자 담당)

| 파일 | 값 | 변경 |
|---|---|---|
| `drumrobot_cfg.py` | `decimation` | 2 → 3 (SIM_DT 0.005 유지) |
| | `action_scale` | 16.6667 → 11.1111 |
| | `wrist_action_scale` | 41.6667 → 27.7778 |
| | `action_delay_steps` | 0 → 1 |
| `hit_detector.py` | `hit_window_step` | 5 → 3 |
| | `alpha` | 0.6 → 0.6314 |
| `skrl_ppo_cfg.yaml` | `discount_factor` | 0.994 → 0.9889 |
| | `rollouts` | 64 → 71 |
| `export_policy.py` | `policy_tick_stride` | 2 → 3 |

---

## 2026-09-02 — 런 2026-09-01_18-01-48 적용 + 스틱 길이 학습 기준 정렬

### 적용한 모델

`logs/skrl/drum_robot/2026-09-01_18-01-48_ppo_torch_drum_robot/checkpoints/best_agent.pt`

학습 설정: `sim.dt 0.0075 x decimation 2 = 15ms (66.7Hz)`, `action_delay_steps 2 (15ms)`,
`action_scale 11.1111`, `wrist_action_scale 55.5556`, `discount_factor 0.991`, `rollouts 64`.

**처음으로 `train_step_dt == policy_dt == 0.015` 가 되어 주기 경고가 사라졌다.**

### 서버 코드 대조 — C++ 은 한 줄도 고치지 않았다

| 항목 | 실기 | export | |
|---|---|---|---|
| `policy_tick_stride` / `policy_dt` | 3 / 0.015 | 3 / 0.015 | OK |
| 관절 순열 | `[0,2,1,5,3,6,4,8,7]` | 동일 | OK |
| 허리·팔 MIT 게인 | kp 100 / kd 5 | 100 / 5 | OK |
| 손목 CST 환산 | 1.4 / 0.14 | 1.4 / 0.14 | OK |
| 정규화 위치 · 좌표계 | 그래프 내부 · 허리기준 m | 동일 | OK |

주기·게인·판정 상수·관절 순서가 전부 `obs_constants.json` 에서 들어오도록 만든 설계가
의도대로 동작했다. `rearm_height` 0.18 -> 0.1, `max_lookahead_step` 120 -> 133 도
자동 반영됐다.

### 변경: `config/kinematics.json` 의 `stick` 0.373 -> 0.385

**학습 기준에 맞춘다는 사용자 결정.**

근거 — 골든 테스트 대조 결과:

```
stick 0.373    tip_pos 12.7mm / hit_armed 38 / per_arm_pos 9 / per_arm_time 3   불일치 4개
stick 0.385    tip_pos  1.4mm / hit_armed  3 / per_arm_pos 0 / per_arm_time 0   불일치 2개
```

12mm 링크 길이 차이 하나가 **네 버퍼의 불일치를 전부 설명**했다. 0.385 로 맞추면
8개 중 6개가 완전 일치하고, `hit_armed` 잔여 3건은 전부 step 0 (상태머신에 이전 팁
값이 없는 초기화 구간), `tip_pos` 잔여 1.4mm 는 학습이 좌우 비대칭 z 오프셋
(-0.023 / -0.026)을 쓰는데 서버 FK 는 평면 스칼라라 생기는 구조적 차이다.

`per_arm_*` 불일치가 step 31 한 곳에서만 났던 것도 같은 원인이었다 — 스케줄러가
팁 위치로 팔을 배정하는데 12.7mm 가 아슬아슬한 배정 하나를 뒤집었다. 스틱을 맞추니
사라졌다. **스케줄러 이식도 정확하다는 뜻이다.**

### 주의 — 이 변경은 개루프 IK 에도 적용된다

`kinematics.json` 은 정책 전용이 아니다. `play_motion_generator` 의 IK, 전이 궤적,
IDLE 이 같은 값을 쓴다. **실제 스틱이 373mm 라면 개루프 경로가 12mm 어긋나게 된다.**

어느 쪽이 물리적으로 맞는지는 실측으로만 확정된다 (손목 회전축 중심 -> 스틱 타격점).
학습 쪽 값이 좌우 비대칭이라 측정값으로 보이지만, 확인된 바 없다.

IK 왕복 검증을 세 번 시도했으나 시험 코드가 매번 틀려(작업공간 밖 각도, 도/라디안
혼동, 파서 오류) **결론을 내지 못했다.** 확실한 것은: 두 길이가 모든 시험에서 동일하게
거동했고, 골든 테스트가 FK 는 0.385 에서 맞다고 말한다.

### 검증

```
빌드         경고 0 · 에러 0
ort_check    입력 9 / 출력 1 규약 일치, 평균 13.8us 최대 19.5us / 예산 15,000us
obs_check    build 성공 400/400, p99 1.57us
golden       8개 중 6개 오차 0
PyTorch/ORT  최대 오차 1.07e-06
```

기존 `data/policy/*` 는 `scratchpad/data_policy_backup/` 에 백업.

### 2026-09-02 정정 — stick 을 0.373 으로 되돌림

위에서 "학습 기준에 맞춘다"며 0.385 로 바꿨으나, **추론이 틀렸다.**

골든 테스트는 **이식 충실도**를 재는 것이지 실제 치수를 알려주지 않는다.
sim 이 385mm 스틱을 모델링하므로 C++ 도 385 로 계산해야 골든이 통과하지만,
그건 "내 C++ 이 sim 과 같은 값을 만든다"는 뜻일 뿐이다.

실제 스틱이 373mm 라면 0.385 로 두는 것은 **틀린 sim 에 맞추려고 실기에
일부러 12mm 오차를 넣는 것**이 된다. 그 경우:

```
개루프 IK : 손목을 목표에서 385mm 뒤에 놓는다 -> 팁은 목표에서 12mm 못 미침
정책      : FK 가 팁을 실제보다 12mm 앞으로 보고 -> 역시 12mm 짧게 침
```

둘 다 같은 방향으로 짧아진다. 수치 확인 결과 **자세와 무관하게 정확히 12.0mm**,
스틱 축 방향으로 일정하다 (드럼 접촉 반경 130mm 의 9%).

그래서 실측 전까지 **실기가 실제로 연주해 온 값 0.373 을 유지**한다.
`kinematics.json` 은 HEAD 와 동일한 상태로 돌아갔다.

**확정 방법** — 0단계에서 `HIT|snare` 타격 위치를 본다:

| 결과 | 뜻 | 조치 |
|---|---|---|
| 중앙을 잘 침 | 373 이 맞다 | 유지. 학습 `specs.py` 의 `tip_offset` 을 0.373 으로 고쳐 재학습 |
| 12mm 깊게/넘어가게 침 | 385 가 맞다 | `kinematics.json` 73줄을 0.385 로. 재학습 불필요 |

줄자로 손목 회전축 중심 -> 스틱 타격점을 재면 더 확실하다.

### IK 왕복 검증 — 결론 (앞의 "결론 내지 못했다" 를 대체)

**IK 는 건전하다.** 앞선 세 번의 실패는 전부 실행 디렉터리 문제였다 —
솔버가 `drumrobot_server/config/kinematics.json` 을 repo 루트 기준으로 열기
때문에 다른 곳에서 실행하면 링크 길이가 0 으로 남아 FK 가 (0,0,0) 을 반환했다.

repo 루트에서 다시 돌린 결과:

```
init/home/ready/shutdown 4개 자세  ->  전 관절 0.000도 일치, FK(IK(x)) 재현 오차 ~1e-16 m
드럼 타격 자세 5개                ->  IK 왕복 오차 1e-16 m, 실패 0건
```

부호 반전 없음. 관절 한계 전부 통과.
