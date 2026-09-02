# 정책 스레드 이식 명세 (drumrobot_server)

> 작성 2026-08-31 · 기준 HEAD `8298e3b` · 웹 버전 https://claude.ai/code/artifact/f44eac63-b072-496f-9407-81a01a7fb99b

---

## 00. 인수인계 브리핑

### 한 줄 요약
드럼 로봇 실기 서버는 지금 **악보 → IK → 궤적을 미리 생성해 재생**하는 오픈루프다.
이걸 **매 스텝 모터 실측을 읽고 추론하는 폐루프 강화학습 정책**으로 바꾼다.
단, 팔(허리·어깨·팔꿈치·손목 9관절)의 **연주 구간만**. 머리·발·전이 궤적은 지금 코드를 그대로 쓴다.

### 두 레포
| 역할 | 경로 | 비고 |
|---|---|---|
| **실기 서버** (수정 대상) | `/home/shy/RL_Real/Phil-drum-robot/drumrobot_server` | C++ · HEAD `8298e3b`. `/home/shy/Phil-drum-robot`은 낡은 사본이니 쓰지 말 것 |
| **학습** (정책 출처) | `/home/shy/RL_workspace/IsaacLab/source/extensions/drum_robot` | Isaac Lab + skrl PPO. **규약의 진실은 이쪽** |

### 런타임 흐름 — PLAY 한 곡
```
[TCP] PLAY|곡id
   ↓
BehaviorPlanner   악보 파싱 → MotionPrimitive 시퀀스 + PolicyScore 발행
   ↓
DRUM/START        planner가 팔 포함 4초 코사인으로 첫 악기 위 자세로 이동   (정책 꺼짐)
   ↓              정지 확인 → MIT 진입 → 게인 0→1 램프 0.5s
   ↓              ctx.policy_active = true
   ↓
DRUM/PLAYING      planner: 발·머리만 생성 → ControlQueue (100ms 버퍼, 팔은 NONE)
   ║              send_loop: 5ms pop → tick++ → t_score 발행 → 3틱마다 cv notify
   ║                         팔 q[0..8] ← PolicyTarget 슬롯으로 덮어쓰기
   ║              policy_thread: 실측 q 스냅샷 → obs148 → 추론 → 슬롯 write (66.7Hz)
   ↓
종료 / 워치독 / abort
   ↓              게인 램프다운 → sync_last_q_from_robot() → policy_active = false
   ↓              MIT 이탈 → 서보 복귀
DRUM/END          planner가 팔 포함 4초로 ready_pose 복귀              (정책 꺼짐)
```
진입·복귀를 planner에 남기는 건 겹침이 아니라 **순서**다. MIT 게인 램프는 정지 상태를 전제로 하고,
정책은 "임의 자세 → 첫 드럼"이 학습 분포 밖이다.

### 확정된 것 (재논의 금지)
- **폐루프.** 정책은 `ControlQueue`에 넣지 않고 최신값 슬롯에 쓴다. 100ms 버퍼를 타면 폐루프가 아니다.
- **정책 66.7Hz(3틱), `POLICY_DT = 0.015`.** 학습은 60Hz. 재학습 없이 간다 — 근거 02절.
- **팔 TMotor는 MIT 모드로 전환**(현재 서보 속도 모드). MIT 식이 학습 환경 PD와 글자 그대로 같다.
- **팔 게인 `Kp=100 / Kd=5`**(TMotor 7개). 학습 쪽 액추에이터도 같은 값으로 이미 수정됨. **손목은 이 정합에 포함되지 않았고 미해결이다 — 아래 참조.**
- **`base`·`state` 모션 생성기의 PLAYING 호출과 `solve_ik`를 제거**한다. 정책이 위치와 타격을 모두 학습했다.

### 아직 열려 있는 것 (임의로 정하지 말 것)
- **손목 게인이 학습/실기 간에 어긋나 있다.** 실기 담당자 확인 대기 — 임의로 어느 쪽도 고치지 말 것.
  | | control_kp | control_kd | 관절 환산 (×gear35/1000) | **Kp/Kd 비** |
  |---|---|---|---|---|
  | 학습 `drumrobot_cfg.py` | 60 | 1 | stiffness 2.1 · damping 0.035 | **60** |
  | 실기 `motors.json` | 40 | 4 | stiffness 1.4 · damping 0.14 | **10** |

  액션이 증분이라 `qd_max = (Kp/Kd) × a_scale × dt`. 손목은 **25.0 vs 4.17 rad/s**.
  학습된 세기 목표는 `dyn_wrist p/mf/f = 3.86 / 7.14 / 10.88 rad/s`이므로
  **1.4/0.14로 학습을 옮기면 중간·셈이 물리적으로 도달 불가**가 된다 — fine-tune으로 흡수되지 않는다.
  (팔의 100/5는 200/10과 Kp/Kd 비가 20으로 동일해 천장이 보존됐다. 그래서 안전했다.)
  실기 이력: `50/0.5` → `60/1`(6/23) → `40/4`(8/18, `8298e3b`). 3회 개정 = 튜닝 이력으로 보인다.
- **손목 중력 보상.** `cal_torque()`의 중력 보상이 학습 환경에는 없다 — 켜두면 이중 보상. 함정 5.
- 실기 `open hihat`(악기 id 9)이 학습 쪽에 없다. `closed hihat`(5)로 접을지 결정 필요.
- 드럼 Z 오프셋 −45mm의 정체(스틱 팁 원점 vs 접촉면). 일률적이라 상수화하면 진행에는 무해.
- LibTorch vs ONNX Runtime. 현재 문서는 LibTorch 전제.

### 규약 검증은 기존 명령으로 한다 (신규 코드 0줄)
정책·obs_builder·MIT를 전부 배제한 상태에서 **관절 순서·각도 규약·FK·드럼 좌표**를 먼저 확인해야 한다.
과거 이걸 위해 CSV 재생(`REPLAY`) 기능을 만들었으나 **불필요하다** — `MOVE` 명령이 이미
임의의 `(모터이름, 각도)` 쌍을 여러 개 받는다(`behavior_planner.cpp` MOVE 핸들러).

```
학습에서 8개 드럼 각각의 "타격 순간 9관절 각도"를 덤프
  → MOVE|waist|10.5|right_shoulder_1|85.2|...|3.0  로 전송
  → 스틱 팁이 해당 드럼 위에 실제로 오는지 자로 확인
```
정적이라 타이밍 교란이 없어 기하 검증에는 오히려 낫다. 클라이언트에 관절각 입력 메뉴도 이미 있다.

---

## 01. 무엇을 대체하는가

정책이 대체하는 것은 **연주 중(`PlayFlag::PLAYING`)의 팔 9관절뿐**이다.

### 팔 경로는 둘로 되어 있다
| 생성기 | 만드는 것 | 정책 구간 |
|---|---|---|
| `base_motion_generator` | **어디로 갈지** — 타격점 좌표·허리각 → `solve_ik` → `q[0..8]` | **제거** |
| `state_motion_generator` | **어떻게 때릴지** — 팔꿈치 lift 15°, 손목 stay 20°/press −5°/lift 40°, `REST_TO_HIT` 등 4상태, 세기별 타이밍. `q[4],q[6],q[7],q[8]`에 가산 | **제거** |
| `pedal_motion_generator` | 발 `q[9],q[10]` | 유지 |
| `head_motion_generator` | 머리 `q[11],q[12]` | 유지 |

**state를 남기면 안 되는 이유**: `state_motion_generator`는 **타격 동작 그 자체**다. 정책은 이걸 포함해
end-to-end로 학습했으므로 남겨두면 하드코딩 스트로크가 정책 스트로크 위에 겹친다.
머지가 `q[0..8]`을 덮어쓰니 최종 지령은 같아 보이지만 **그렇지 않다** —
`if (base.get_error() || state.get_error()) return empty;` 와 `solve_ik` 실패가 빈 큐를 반환해
`ctx.play_abort = true`로 이어진다. **정책은 IK를 쓰지도 않는데 IK 실패가 연주를 중단시킨다.**

### 관절 소유권 (분할선은 관절이 아니라 모드)
| 상태 / MotionType | 팔 id 0–8 | 머리 11–12 · 발 9–10 |
|---|---|---|
| STANDBY · INIT · IDLE | MotionPlanner | MotionPlanner |
| TRANSLATE (POSE·MOVE·LOOK) | MotionPlanner | MotionPlanner |
| DRUM / START (진입 4초) | MotionPlanner | MotionPlanner |
| **DRUM / PLAYING** | **Policy** | MotionPlanner |
| DRUM / END (복귀 4초) | MotionPlanner | MotionPlanner |

`TrajectoryGenerator`의 궤적 생성 함수 6개 중 5개가 팔을 움직인다
(`standby`, `joint/task_space`, `play_start`, `play_end`, `idle`).
팔을 통째로 빼면 로봇이 ready pose로 진입도 복귀도 못 한다.

### 정책은 큐에 넣지 않는다
`ControlQueue`는 13축 통짜 setpoint의 단일 생산자 FIFO이고 `MotionPlanner`가 잔량 20개(≈100ms) 이하일 때 미리 채운다.
정책 출력을 이 큐에 넣으면 **100ms 버퍼를 타게 되어 폐루프가 성립하지 않는다.**
정책은 큐가 아니라 **최신값 슬롯**(seqlock)에 쓰고, `Controller::send_loop`이 pop한 setpoint의 `q[0..8]`을 덮어쓴다.
생산자는 여전히 MotionPlanner 하나이므로 큐를 슬라이스로 쪼갤 필요가 없다.

### 겹침을 구조로 막는다 — 단일 가드
```cpp
void TrajectoryGenerator::push_setpoint(ControlSetPoint& sp) {
    if (ctx.policy_active.load()) {
        for (int j = 0; j < 9; ++j) {
            sp.q[j] = 0.0; sp.qd[j] = 0.0;
            sp.mode[j] = ControlMode::NONE;   // "이 관절은 내 소유가 아님"
        }
    }
    control_queue.push(sp);
}
```
`control_queue.push()` 직접 호출을 **전부** 이걸로 교체한다 —
`standby` / `joint_space` / `task_space` / `play_start` / `play_end` / `idle` / `play`.
한 군데라도 빠지면 가드가 새는 지점이 된다.

**NONE이 곧 fail-safe다.** `tmotor_send_task`·`maxon_motor_send_task`가 이미 `NONE`을 만나면
로그를 찍고 `continue`한다 → **프레임 미송신 = 모터가 직전 명령 유지**.
머지가 어떤 이유로든 실패해도 팔은 자동 홀드된다. (200Hz `cerr` 스팸만 rate-limit 할 것)

---

## 02. 제어 주기

| 계층 | 주기 | 주체 | 변경 |
|---|---|---|---|
| CAN 수신 | 100 µs | `recv_loop` | 스냅샷 발행만 추가 |
| Maxon 보간 송신 | 1 ms | `send_loop` 내부 | 없음 |
| **제어 setpoint** | **5 ms · 200 Hz** | `send_loop` pop = **tick** | tick·머지 추가 |
| **정책 추론** | **15 ms · 66.7 Hz** | `policy_thread` | 신규 |
| 궤적 생성 | 비주기 (잔량<20) | `MotionPlanner` | play 경로만 |

**기존 하드웨어 루프는 하나도 안 바뀐다.** 위에 66.7Hz 계층 하나가 얹힐 뿐이다.

### 왜 15ms인가
학습은 60Hz(`SIM_DT=1/120`, `decimation=2`)였다. 실기는 200Hz라 200/60 = 3.33으로 정수가 아니다.
**3틱(15ms, 66.7Hz)**으로 가고 적분에 실제 dt를 쓴다. 4틱(20ms, 50Hz)은 60Hz에서 더 멀다.
```cpp
constexpr uint64_t POLICY_TICK_STRIDE = 3;      // 3 × 5ms
constexpr double   POLICY_DT          = 0.015;  // ★ 1.0/60.0 하드코딩 금지
```
1.67ms 차이가 안전한 이유: ① 액션이 속도 지령(`q_des = q_now + a·scale·dt`)이라 실제 dt를 쓰면
지령 각속도가 보존된다 ② 학습 obs 시간 채널이 `offset/(L−1)`로 정규화돼 `step_dt`가 약분된다.
**따라서 스텝 단위 상수는 전부 초 단위로 포팅해야 이 불변성이 성립한다** (04절).

### 틱 위상
```
tick N   (0ms)   pop → tick++ → t_score 발행 → cv notify → 정책 계산(~0.15ms)
                 이 틱이 모터로 보내는 팔 값 = tick N−3 결과
tick N+1 (5ms)   새 정책 출력 적용 ← 여기서부터
tick N+2 (10ms)  ZOH
tick N+3 (15ms)  다음 정책 출력
```
팔은 **ZOH(3틱 유지)** — 학습이 `decimation` 동안 타겟을 홀드하는 것과 같은 구조.
보간을 넣으면 오히려 학습과 달라진다. 손목(CST)만 기존 Maxon 1ms 보간을 탄다.

파이프라인 지연 1틱(5ms)은 구조적으로 불가피하다. 총 예산:
`recv 0.1 + 추론 0.15 + 파이프라인 5.0 + CAN 2.5 ≈ 7.8ms` → 학습 쪽 `action_delay_steps=1`(8.3ms)이
보수적 상한이 아니라 **정확한 중심값**이다.

### 시간 기준이 둘이다
`play_speed_scale`이 1.0이 아닐 때 갈린다.
- **악보 시간** `ctx.t_score` — 다음 노트가 뭔지 찾는 데 쓴다. `round_sum` 반올림 보정과 배속, pause/resume이 이미 반영돼 있다.
- **실시간** — `time_to_hit` obs 정규화에 쓴다. 물리는 배속되지 않는다.
  `t_to_hit_real = (t_score_of_next − ctx.t_score) / ctx.play_speed_scale`

---

## 03. 함정 여섯

### 함정 1 — `last_q` 연속성
`TrajectoryGenerator`는 다음 구간의 시작점으로 `last_q`를 쓴다. PLAYING 동안 팔은 정책이 움직이는데
`TrajectoryGenerator`는 그걸 모른다. 그래서 END 시점의 `last_q`는 **START 때 멈춘 낡은 값**이고,
복귀 궤적 첫 setpoint가 실제 자세에서 수십 도 떨어진 곳으로 점프한다.
`POS_DIFF_LIMIT`(30°)가 프레임을 버려 최악은 막지만 그건 안전망이지 해결이 아니다.
→ `policy_active`를 내리기 **직전에** 실측 q를 되쓴다.

### 함정 2 — 머리 yaw가 허리에 묶여 있다
`src/trajectory/play_motion_generator.cpp:180`
```cpp
q[11] = h.yaw - q[0];   // 머리 yaw를 허리각으로 보정
```
→ planner는 **raw `h.yaw`**를 내보내고, `send_loop` 머지 지점에서 `q[HEAD_YAW] -= pt.q[0]`.
나머지 두 결합(`next_note = rds[1].note_num_R`, `nod_intensity ← velocity_R/L·is_kick`)은
**전부 악보에서 오므로 문제가 아니다** — 정책은 같은 악보를 따를 뿐 악보를 대체하지 않는다.
`LOOK` 경로는 `q[HEAD_YAW]`를 직접 쓰고 허리 보정을 하지 않으므로 raw 규약은 **PLAYING 경로에만** 적용한다.

### 함정 3 — `v_des = 0`으로 보내야 한다
```
학습:  τ = Kp(q_des − q) + Kd(0 − qd)      ← Kd가 순수 감쇠
MIT :  τ = Kp(p_des − p) + Kd(v_des − qd)
```
학습 환경은 `set_joint_position_target`만 호출하고 속도 타겟은 건드리지 않는다(기본값 0).
`v_des`에 정책 속도를 넣으면 **감쇠항이 추종항으로 바뀐다.** `v_des = 0`, `t_ff = 0`.

### 함정 4 — 모터 상태의 시간 정합성
지금까지 `robot.motors[id]->current_joint_angle`은 `recv_loop`이 쓰고 아무도 동시에 읽지 않았다.
이제 정책이 읽는다. `double`이라 tearing은 실질적으로 없지만, 정책이 9관절을 순회하는 동안
100µs 주기의 recv가 중간을 갱신하면 **관절 0은 t, 관절 5는 t+50µs**인 뒤섞인 상태가 obs로 들어간다.
→ `distribute_frames()` 끝에서 9관절을 **한 번에** 스냅샷으로 발행(seqlock/더블버퍼).

### 함정 5 — 손목 중력 보상 제거 *(미확정 — 실기 담당자 확인 필요)*
**손목 모드는 손댈 필요 없다.** `get_modes(is_play)`가 이미 연주 중 `CST`, 그 외 `CSP`로 전환한다
(`trajectory_generator.cpp:286`). 즉 `control_kp/kd`와 `cal_torque()`는 연주 중 실제로 실행되고 있다.

남는 문제는 **중력 보상**이다. `cal_torque()`에 스틱 중력 보상항이 있는데 **학습 환경에는 없다** —
학습 물리에 실제 중력이 있고 정책이 스스로 이겨내도록 학습됐다.
실기에서 켜면 이중 보상이 되어 스틱이 위로 들린다. → 정책 채널에서는 꺼야 한다.

그리고 **손목 게인 불일치**(00절)가 여기 걸린다. 법칙(CST)은 이미 맞는데 계수가 다르다.

### 함정 6 — `q` 배열이 초기화되지 않는다
`generate_motion()`의 `std::array<double,13> q;`는 **초기화 없이** 선언되고 원본은 13개를 전부 채웠다.
팔 계산을 빼면 `q[0..8]`이 쓰레기 값으로 남는다. `q{}`로 0 초기화하고 그 위에 `ControlMode::NONE` 가드를 얹는다.
**0 역시 팔에겐 위험한 자세이므로 값이 아니라 모드로 막는 것이 요점이다.**

---

## 04. 정책 계약

### 입출력
```
obs[148] ──> RunningStandardScaler ──> MLP 256-160-128 (elu) ──> act[9] ∈ [-1,1]

q_tgt[j] = q_now[j] + act[j] × scale[j] × POLICY_DT
           scale = 10.0 rad/s (팔 0~6) · 25.0 rad/s (손목 7,8)
           POLICY_DT = 0.015
```

### obs 148 레이아웃 — 이 순서 그대로
| # | 구간 | 크기 | 정규화 | 실기 조달 |
|---|---|---|---|---|
| 1 | `joint_pos` | 9 | `(q−center)/half_range`, clamp ±1.5 | 엔코더 |
| 2 | `joint_vel` | 9 | `/5.0`, clamp ±10 | TMotor 피드백 · 손목은 유한차분 |
| 3 | `tip_pos` | 6 | `(p−drum_center)/drum_half_range`, clamp ±1.5 | `solve_fk(q9)` 재사용 |
| 4 | `drum_pos` | 24 | 위와 동일 | `drum_coordinate.json` + 변환 |
| 5 | `next_hits` | 66 | K6 × (onehot8 + t + valid + u) | `PolicyScore` |
| 6 | `hit_armed` | 16 | 0/1 · 2팔 × 8드럼 | **온보드 상태머신** |
| 7 | `arm_role` | 2 | 활성 1 / 노는팔 0 | 스케줄러 파생 |
| 8 | `per_arm_feat` | 16 | 2팔 × 2노트 × (pos3 + time1) | **스케줄러** |

합 = 9+9+6+24+66+16+2+16 = **148**.
`next_hits`의 time 채널은 `t_to_hit_real / 2.0`(`max_lookahead_s`), 세기 채널 `u`는 `DrumEvent.velocity / 127`.

### 스텝 단위 상수는 전부 초로 옮긴다
| 학습 | 값 | C++ |
|---|---|---|
| `hit_window_step` | 3 스텝 | `HIT_WINDOW_S = 0.05` |
| `t_rearm = 2W+1` | 7 스텝 | `REARM_S = 0.1167` |
| `max_lookahead_step` | 120 | `max_lookahead_s = 2.0` 직접 사용 |
| `time_norm = offset/(L−1)` | — | `t_sec / max_lookahead_s` |

### 좌표 규약 — 전수 확인 완료
학습 쪽 드럼 위치는 실기 `right`/`left` 좌표의 **중점**이고, XY는 8개 악기 전부 0.5mm(반올림) 이내로 일치.
**Z만 일률적으로 −45mm** 차이.
```
obs용 drum_pos = midpoint(real.right, real.left) + (0, 0, +0.045)
```
실기의 팔별 x 분리(±11~52mm)는 학습 쪽 `aim_offset = 0.04`가 같은 역할을 한다. 재학습 불필요.

### 악기 매핑
`instrument_name_to_id`(실기)와 학습 쪽 악기 id가 **1~8 완전 일치**
(`top`=high, `closed hihat`=hihat, `right/left crash`=crash_r/l).
```
obs 인덱스 = real_id − 1        // snare 1 → 0 … left crash 8 → 7
real_id 0 (bass)        → 페달 채널, obs 아님
real_id 9 (open hihat)  → 미결정
```

### export 3종 (학습 레포에서 뽑는다)
1. **TorchScript** — GaussianMixin에서 결정적 평균만 trace
2. **`RunningStandardScaler`** mean / var / count
3. **정규화 상수** `joint_center`, `joint_half_range`, `drum_center`, `drum_half_range`

3번을 **반드시 파일로 뽑을 것.** C++에 손으로 옮기면 학습 쪽에서 관절 한계나 드럼 위치를 만질 때 조용히 어긋난다.

---

## 05. 신규 파일

| 파일 | 역할 |
|---|---|
| `include/policy/policy_target.hpp` | 정책 출력 **최신값 슬롯**(seqlock). `q[9]` + `tick` + `seq` + `valid`. 큐가 아니다 |
| `include/policy/policy_score.hpp` | 정책용 악보 스냅샷. `vector<{t_sec, drum_mask, velocity}>`. 곡 시작 시 1회 발행 |
| `src/policy/policy_runner.cpp` | 스레드 본체. TorchScript 로드 → cv 대기 → 추론 → 적분 → 슬롯 write |
| `src/policy/obs_builder.cpp` | **작업량의 절반.** 148차원 조립 + 정규화. 재장전 상태머신 포함 |
| `src/policy/normalizer.cpp` | `RunningStandardScaler` 적용 |

### policy_runner 루프
```cpp
while (ctx.running) {
  std::unique_lock lk(policy_mtx);
  if (policy_cv.wait_for(lk, 50ms) == std::cv_status::timeout) { ctx.policy_fault = true; continue; }
  if (!ctx.policy_active) continue;

  auto st = robot.joint_snapshot();          // q[9], qd[9] 일괄 (함정 4)

  obs_builder.build(st, ctx.t_score.load(), ctx.play_speed_scale.load(), obs);
  normalizer.apply(obs);
  net.forward(obs, act);                     // ~0.03ms

  for (int j = 0; j < 9; ++j)
      q_tgt[j] = clamp(st.q[j] + clamp(act[j],-1,1) * scale[j] * POLICY_DT, lo[j], hi[j]);
  policy_target.publish(q_tgt, ctx.tick.load());
}
```

### SCHED_FIFO에서 LibTorch
실시간 우선순위로 도는 스레드에서 스텝마다 동적 할당이 일어나면 malloc의 futex에서 우선순위 역전이 날 수 있다.
예산 15ms 중 0.15ms만 쓰지만 **역전은 여유와 무관하게 터진다.**
`torch::set_num_threads(1)`, `torch::NoGradGuard`, 입출력 텐서 사전할당 후 재사용,
루프 내 `.clone()`/`.to()` 금지.

---

## 06. 수정 파일

### `include/common/robot_config.hpp` — JointID 이동
현재 `JointID`는 `src/trajectory/behavior_planner.cpp:4–16`에 있다(헤더가 아님).
`Controller`가 `JointID::HEAD_YAW`를 써야 하는데 접근할 수 없다. 여기로 옮기고 원래 정의는 지운다.
```cpp
namespace JointID {
    constexpr int WAIST = 0;
    // ... 1~10 ...
    constexpr int HEAD_YAW = 11;
    constexpr int HEAD_PITCH = 12;
    constexpr int NUM_ARM = 9;      // 정책 담당 0~8
}
```

### `include/common/control_queue.hpp`
```cpp
enum class ControlMode {
    POS, VEL,
    MIT,              // ★ 신규 — TMotor MIT 모드
    CST, CSV, CSP,
    NONE,
};

struct ControlSetPoint {
    std::array<double, ROBOT::NUM_JOINT> q{}, qd{};
    std::array<ControlMode, ROBOT::NUM_JOINT> mode{};
    bool   audio_start = false;
    double t_score    = 0.0;   // ★ 신규 — 이 setpoint의 악보 시간 [s]
};
```
`t_score`를 여기 싣는 이유: `get_num_point()`가 `round_sum` 누산기로 반올림 오차를 흡수하고
`play_speed_scale`까지 반영한다. 정책이 `tick × 5ms`로 시간을 따로 계산하면 배속 재생에서
팔과 발이 어긋난다. 시간의 소유자를 문자 그대로 하나로 만든다.

### `include/common/app_context.hpp`
```cpp
struct AppContext {
    // ... 기존 필드 유지 ...
    std::atomic<uint64_t> tick{0};          // send_loop이 5ms pop마다 +1. 유일한 시간 권위
    std::atomic<double>   t_score{0.0};     // 현재 tick의 악보 시간
    std::atomic<bool>     policy_active{false};
    std::atomic<bool>     policy_fault{false};
    std::atomic<double>   gain_ramp{0.0};   // MIT 게인 0→1 코사인 램프

    std::mutex              policy_mtx;
    std::condition_variable policy_cv;      // send_loop이 3틱마다 notify
};
```

### `include/hardware/motor_codec.hpp` · `src/hardware/motor_codec.cpp` — 🐛 버그 수정
`TMotorMITCodec`은 완성돼 있지만 **한 번도 인스턴스화되지 않은 죽은 코드**이고, 초기화되지 않은 멤버가 있다.
```cpp
float GLOBAL_P_MIN = -12.5, GLOBAL_P_MAX = 12.5;    // OK
float GLOBAL_KP_MIN = 0, GLOBAL_KP_MAX = 500;       // OK
float GLOBAL_KD_MIN = 0, GLOBAL_KD_MAX = 5;         // OK
float GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX;  // ✗ 대입되는 곳이 없음
float GLOBAL_I_MAX, Kt;                                        // ✗ 읽히기만 함
```
`encodeCommand`이 이 값들로 clamp하고 12비트로 인코딩한다. 그대로 쓰면 `v_des`·`t_ff`가
쓰레기 값으로 잘리고 프레임이 깨진다. 게다가 **AK10-9(허리)와 AK70-10(팔)이 다르므로**
클래스 전역이 아니라 모터별이어야 한다.
```cpp
void encodeCommand(can_frame* f, int canId, int dlc,
                   float p_des, float v_des, float kp, float kd, float t_ff,
                   const MotorMitLimits& lim);   // ★ 시그니처 변경
```

### `include/hardware/motor.hpp` · `config/motors.json`
```cpp
class TMotor : public Motor {
    double control_gain;      // 기존 (서보 VEL 모드용) — 유지
    double mit_kp, mit_kd;                     // ★ 신규
    double mit_v_limit, mit_t_limit, mit_kt;   // ★ 신규 (인코딩 범위)
};
```
| 관절 | mit_kp | mit_kd | ζ |
|---|---|---|---|
| waist (AK10-9) | 100 | 5 | 0.302 / f_n 1.92 Hz |
| shoulder ×4 (AK70-10) | 100 | 5 | 0.804 |
| elbow ×2 (AK70-10) | 100 | 5 | 1.605 |

학습 쪽 액추에이터 게인과 **정확히 같은 값**이어야 한다. 매뉴얼 상한은 `Kd ≤ 5`.

### `include/realtime/controller.hpp` · `src/realtime/controller.cpp` — 가장 많이 바뀜

**(a) `send_loop` — `cnt == 0` 블록**
```cpp
if (cnt == 0) {
    prev_point = curr_point;
    if (auto sp = control_queue.try_pop()) {
        curr_point = *sp;
        if (curr_point.audio_start) audio_player.play();
        ctx.t_score.store(curr_point.t_score);       // ★ 악보 시간 발행
    } else { /* 기존 underflow 로그 */ }

    uint64_t t = ctx.tick.fetch_add(1) + 1;          // ★ 단일 클록

    if (ctx.policy_active.load()) {
        PolicyTarget pt = policy_target.snapshot();
        if (pt.valid && (t - pt.tick) <= WATCHDOG_TICKS) {   // 6틱 = 30ms
            for (int j = 0; j < 9; ++j) curr_point.q[j] = pt.q[j];
            for (int j = 0; j < 7; ++j) curr_point.mode[j] = ControlMode::MIT;
            curr_point.mode[7] = curr_point.mode[8] = ControlMode::CST;   // 함정 5
            curr_point.q[JointID::HEAD_YAW] -= pt.q[0];                   // ★ 함정 2
        } else {
            // 팔 모드는 planner가 넣은 NONE 그대로 둔다 → 프레임 미송신 → 모터 홀드
            ctx.policy_fault = true;
            ctx.play_abort   = true;
        }
        if (t % POLICY_TICK_STRIDE == 0) ctx.policy_cv.notify_one();
    }
    send_task_1ms(cnt);
}
```
`9~12`(발·머리)는 MotionPlanner가 준 값을 그대로 둔다.
워치독 분기에서는 **아무것도 하지 않는 것이 정답**이다 — planner가 넣은 `ControlMode::NONE`이 남아
프레임이 나가지 않고 모터가 홀드한다(01절). 기존 안전 가드(`POS_DIFF_LIMIT` 30°, min/max, 전류)는 그대로 걸린다.

**(b) `tmotor_send_task` — MIT 분기**
```cpp
if (mode == ControlMode::MIT) {
    mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
        tmotor->joint_angle_to_motor_position(point.q[id]),   // p_des
        0.0f,                                                // ★ v_des = 0 (함정 3)
        tmotor->mit_kp * ctx.gain_ramp.load(),               // ★ 램프
        tmotor->mit_kd * ctx.gain_ramp.load(),
        0.0f,                                                // t_ff = 0
        tmotor->mit_limits());
    robot.can.sendFrame(tmotor->socket, frame);
}
```
5ms 송신을 유지한다 — `send_task_1ms`의 보간은 Maxon 전용이고, 팔의 ZOH가 학습 `decimation` 구조와 일치한다.

**(c) `recv_loop` — `distribute_frames()` 끝에 9관절 일괄 스냅샷 발행** (함정 4).
손목은 Maxon이라 속도 피드백이 없으므로 `POLICY_DT` 기준 유한차분으로 채운다.

### `src/trajectory/play_motion_generator.cpp`
`generate_motion()` · 125–184행. **`reset()`은 건드리지 않는다.**
```cpp
  auto [n, dt] = get_num_point(rds[0].t, rds[1].t);   // 유지 — 시간의 소유자

- std::queue<BaseMotionPoint>  base_motion  = base_motion_generator.generate_motion(rds, n, dt);
- std::queue<StateMotionPoint> state_motion = state_motion_generator.generate_motion(rds, n, dt);
  std::queue<HeadMotionPoint>  head_motion  = head_motion_generator.generate_motion(rds, n);
  std::queue<PedalMotionPoint> pedal_motion = pedal_motion_generator.generate_motion(rds, n, dt);

- if (base_motion_generator.get_error() || state_motion_generator.get_error()) return empty;
  // ★ 이 게이트를 지우는 것이 핵심 — IK·state 실패가 play_abort로 이어지던 결합을 끊는다

  for (int i = 0; i < n; i++) {
      std::array<double, ROBOT::NUM_JOINT> q{};   // ★ zero-init 필수 (함정 6)

-     KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, theta0, theta7, theta8, true);
-     if (!result.success) return empty;
-     for (int i = 0; i < 9; i++) q[i] = result.q[i];
-     q[4] += s.right_elbow;  q[6] += s.left_elbow;
-     q[7] += s.right_wrist;  q[8] += s.left_wrist;
      // 팔 0~8은 아무것도 쓰지 않는다. send_loop이 정책 값으로 덮어쓴다.

      q[9]  = p.right;              // 발 — 그대로
      q[10] = p.left;
-     q[11] = h.yaw - q[0];
      q[11] = h.yaw;                // ★ raw. 허리 보정은 머지 지점에서 (함정 2)
      q[12] = h.pitch;
      q_queue.push(q);
  }
```
각 setpoint에 `t_score`를 채운다.
`base_motion_generator`·`state_motion_generator` **파일은 지우지 않는다** —
`PlayMotionGenerator::reset()`이 연주 진입 자세에 계속 쓰고, 2단계 MIT 검증에서 기존 연주를 돌릴 때도 필요하다.

### `include/trajectory/trajectory_generator.hpp` · `.cpp`
```cpp
public:
    void sync_last_q_from_robot(const Robot& robot);   // ★ 신규 (함정 1)
private:
    void push_setpoint(ControlSetPoint& sp);           // ★ 신규 — 팔 소유권 가드 (01절)
    // wrist_control_mode 는 손대지 않는다 — get_modes(is_play)가 이미 연주 중 CST로 전환한다
```
`control_queue.push()` 직접 호출을 **전부** `push_setpoint()`로 교체한다 — 여섯 군데.
`sync_last_q_from_robot`은 실측 `q[0..8]`을 `last_q`에 되쓴다. `9~12`는 건드리지 않는다.
`cal_torque()`의 중력 보상은 정책 구간에서 꺼야 한다(미확정 — 함정 5).

### `src/trajectory/behavior_planner.cpp`
`JointID` 정의를 `robot_config.hpp`로 옮긴 뒤 여기서 지운다. PLAY 시퀀스에 `PolicyScore` 발행과 모드 전환 절차를 넣는다.

**MIT 전환 시퀀스**
1. START 궤적으로 ready pose 이동 — 기존 서보 VEL 모드 그대로
2. 정지 확인 (`|qd| < ε`, 전 관절)
3. `encodeExitControlMode` → `encodeEnterControlMode` (id 0~6)
4. `p_des = 현재 실측 q`, `gain_ramp = 0`에서 시작
5. 0.5초 코사인으로 `gain_ramp` 0 → 1 — **안전의 핵심**
6. `ctx.policy_active = true`

**종료(정상 · 워치독 · `play_abort`) 시 역순**
7. `gain_ramp` → 0 램프다운
8. **`trajectory_generator.sync_last_q_from_robot()`** — 함정 1
9. `policy_active = false` → `encodeExitControlMode` → 서보 복귀

4가 중요하다. `p_des`를 목표 자세로 두고 게인을 올리면 그 오차만큼 즉시 토크가 나온다.
**8은 반드시 9보다 먼저**여야 한다.

### `src/main.cpp` (44–57행)
```cpp
PolicyTarget policy_target;
PolicyRunner policy_runner(ctx, policy_target, robot);

std::thread policy_thread(&PolicyRunner::run, &policy_runner);   // ★ 신규

set_priority(send_thread,   40);
set_priority(recv_thread,   30);
set_priority(policy_thread, 25);   // ★ recv보다 낮게, motion보다 높게
set_priority(motion_planning_thread, 20);
set_priority(tcp_server_thread,      10);

policy_thread.join();
```
**recv보다 낮게** 두는 게 중요하다. 정책이 폭주해도 상태 갱신이 굶으면 안 된다.
MotionPlanner보다는 높다 — 궤적 생성은 무겁고 데드라인이 없다.

### `drumrobot_server/Makefile`
`SOURCES`는 `find`로 자동 탐색하므로 새 `.cpp`는 그냥 잡힌다.
**그런데 `directories` 타깃이 obj 하위 폴더를 하드코딩하고 있어** `src/policy/`를 추가하면 빌드가 깨진다.
```make
directories:
	mkdir -p $(BINDIR)
	mkdir -p $(OBJDIR)/common $(OBJDIR)/hardware $(OBJDIR)/kinematics \
	         $(OBJDIR)/tcp $(OBJDIR)/realtime $(OBJDIR)/trajectory \
	         $(OBJDIR)/util $(OBJDIR)/policy          # ★ 추가
	mkdir -p $(OBJDIR)/dxl

# ===== LibTorch =====
TORCH_DIR = ./lib/libtorch
INCLUDE  = -I./include -I./lib -I./lib/dynamixel_sdk/include \
           -I$(TORCH_DIR)/include -I$(TORCH_DIR)/include/torch/csrc/api/include
LDFLAGS  = -lpthread -lm -ldl \
           -L$(TORCH_DIR)/lib -ltorch -ltorch_cpu -lc10 -Wl,-rpath,$(TORCH_DIR)/lib
```
`CFLAGS`의 `-std=c++17`은 LibTorch와 호환된다. CPU 빌드만 필요하다 — MLP 256-160-128은 0.03ms.

---

## 07. 손대지 않는 것

| 대상 | 이유 |
|---|---|
| `pedal_motion_generator.*` | `is_kick`/`is_closed_hihat`만 쓰고 IK도 안 타고 다른 관절을 참조하지 않는다. 깨끗하다 |
| `base_` · `state_motion_generator.*` | **파일은 유지 · PLAYING 호출만 제거.** `reset()`이 연주 진입 자세에 쓰고, 2단계 MIT 검증에서 기존 연주를 돌릴 때도 필요하다 |
| `head_motion_generator.*` | 입력이 전부 악보(`note_num_R`, `velocity_R/L`, `is_kick`, `beat`)다. 정책이 팔을 맡아도 악보는 그대로 있다 |
| `kinematics_solver.cpp` | `solve_fk(q9) → tips`를 `obs_builder`가 **그대로 재사용**한다 |
| `can_interface` · `tcp_server` · `command_parser` | 정책과 무관. `GET_STATUS`에 상태 노출 정도만 선택적 |
| `t_codec` / `m_codec` 기존 인코딩 | 서보 POS/VEL, Maxon CSP/CST/HMM 전부 유지. MIT 분기만 추가 |
| `audio_player` · 전이 궤적 · pause/resume | 그대로. `t_score`를 통해 정책이 자동으로 따라온다 |

---

## 08. 작업 순서

1. **정적 자세 대조** — 학습에서 뽑은 타격 자세를 `MOVE`로 명령하고 스틱 팁 위치를 실측. **신규 코드 없음**
2. **MIT 모드 단독 검증** — 코덱 버그 수정 + 게인 램프 후, `tmotor_control_mode`를 `VEL` → `MIT`로 바꾸고 **기존 하드코딩 연주(`PLAY`)를 그대로 돌린다.** 정책과 섞기 전에 MIT PD가 실제 타격 부하에서 버티는지 확인 — CSV 재생보다 나은 시험이다
3. **학습 게인 정합 후 resume fine-tune** — 학습 액추에이터를 100/5로 맞추고 이어서 학습(이미 수정 완료). 기존 체크포인트는 실기에 존재할 수 없는 `damping=10`에서 나왔다
4. **정책 export 3종** — 04절
5. **`obs_builder` + 골든 테스트** — 학습에서 한 에피소드의 (상태, obs 148)을 덤프해 C++에 같은 상태를 주고 **요소별로 비교**
6. **`policy_runner` + 머지 + 워치독** — CAN 송신을 끄고 로그만. 지령 궤적이 학습 재생과 닮았는지
7. **실기 저게인 · 드럼 비접촉** — E-stop 확보 후 첫 구동
8. **실타격** — 성공률·타이밍을 학습 지표와 대조

**5단계가 승부처다.** 골든 테스트 없이 실기로 가면 "왜 안 되는지"를 구분할 수 없다 —
obs 조립 버그인지, 규약 불일치인지, sim2real 간극인지.
`obs_builder`에서 가장 무거운 부분은 `hit_armed` 재장전 상태머신이다
(팁이 `rearm_height` 위로 올라가야 재장전, 최근접 드럼 중재 포함).
