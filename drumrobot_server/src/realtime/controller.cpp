#include "realtime/controller.hpp"

#include <cmath>       // std::isfinite

Controller::Controller(AppContext &ctxRef, ControlQueue &controlQueueRef, Robot &robotRef,
                       AudioPlayer &audioRef, PolicyTarget &policyTargetRef)
    : ctx(ctxRef), control_queue(controlQueueRef), robot(robotRef), motor_log("motors"),
      motor_ext_log("motors_ext"), audio_player(audioRef), policy_target(policyTargetRef)
{
    for (auto &[id, motor] : robot.motors) {
        if (id < ROBOT::NUM_JOINT) {
            curr_point.q[id] = motor->initial_joint_angle;
            prev_point.q[id] = motor->initial_joint_angle;
        }
    }

    // motors 는 원 레포 헤더 그대로. 열을 늘리면 MATLAB 쪽이 깨진다 — 확장은 motors_ext 로.
    std::vector<std::string> header = {"id", "mode", "desired", "actual", "err", "current/torque", "input"};
    motor_log.set_header(header);

    std::vector<std::string> ext_header = header;
    ext_header.push_back("velocity");
    ext_header.push_back("tick");
    motor_ext_log.set_header(ext_header);
}

Controller::~Controller() {}

void Controller::log_motor_row(const std::vector<double> &values, double velocity) {
    motor_log.record(values);

    std::vector<double> ext = values;
    ext.resize(7, 0.0);                     // id, mode, desired, actual, err, current/torque, input
    ext.push_back(velocity);
    ext.push_back(static_cast<double>(ctx.tick.load()));
    motor_ext_log.record(ext);
}

void Controller::send_loop() {
    int cnt = 0;

    while (ctx.running.load()) {
        // MIT 는 명령에 대한 응답으로만 피드백을 준다. START 의 실측 동기화가
        // current_joint_angle 을 필요로 하므로, recv 가 켜진 시점부터 게인 0 명령을
        // 보내 first_recv_done 을 채워 둔다.
        if (ctx.recv_active.load() && ctx.tmotor_mit.load() && !all_tmotors_received()) {
            prime_mit_feedback();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (!ctx.send_active.load()) {
            if (ctx.robot_state.load() == RobotState::SHUTTINGDOWN) break;  // send_active 전 종료 상태가 되면 바로 탈출
            std::this_thread::sleep_for(std::chrono::milliseconds(10));     // send_active 전까지 대기
            continue;
        }

        if (!all_tmotors_received()) {
            // MIT는 command에 대한 응답으로만 피드백을 준다.
            // 게인 0 명령(= 토크 0)을 보내 응답을 유도한다.
            if (ctx.tmotor_mit.load()) prime_mit_feedback();

            std::this_thread::sleep_for(std::chrono::milliseconds(10));     // 모든 모터가 값을 수신할 때까지 대기
            continue;
        }

        auto next = std::chrono::steady_clock::now();

        while (ctx.running.load() && ctx.send_active.load()) {
            next += std::chrono::microseconds(1000);    // 1ms 주기

            if (cnt == 0) {
                // MIT 제어 모드 진입 요청 처리 (send_thread에서만 CAN에 쓴다)
                if (ctx.mit_enter_requested.exchange(false)) {
                    enter_mit_control_mode();
                }

                // 1ms: 큐에서 새 목표값 가져오고, 맥슨 보간 1번째 송신
                prev_point = curr_point;
                if (auto sp = control_queue.try_pop()) {
                    curr_point = *sp;
                    // 정책 소유 setpoint 의 q[11] 은 아직 보정 전(raw)이다.
                    // merge 가 대입으로 쓸 수 있게 pop 시점에 원본을 보관한다.
                    raw_head_yaw = curr_point.q[JointID::HEAD_YAW];
                    if (curr_point.audio_start) audio_player.play();
                    ctx.t_score.store(curr_point.t_score);      // 악보 시간 발행
                } else {
                    static int err_cnt = 0;
                    if (err_cnt++ % 100 == 0) std::cerr << "[Controller] control_queue underflow\n";
                }

                // 유일한 시간. 정책·머지·워치독이 전부 이 값을 기준으로 한다.
                uint64_t t = ctx.tick.fetch_add(1) + 1;

                // 깨우기는 ctx.policy_active 기준 — 이 플래그는 궤적 생성 시점에 서므로
                // 첫 정책 소유 setpoint 가 도착하기 약 100ms 전부터 정책이 돈다.
                // 그 선행 구간이 슬롯을 채워 두고, ObsBuilder 의 팁·손목 이전값도
                // 예열해 준다 (첫 스텝의 유한차분 속도가 0 이 되지 않는다).
                if ((ctx.policy_active.load() || ctx.policy_dry_run.load())
                    && t % POLICY_TICK_STRIDE == 0) {
                    ctx.policy_cv.notify_one();
                }

                // 머지는 setpoint 가 들고 온 플래그 기준 — 경계에서 어긋나지 않는다.
                if (curr_point.policy_owns_arm) merge_policy_target(t);

                // 송신 전 마지막 관문. merge 뒤라 정책 값·머리 보정까지 반영된 최종 카드를 본다.
                if (!targets_finite(curr_point)) { ctx.running = false; break; }

                send_task_1ms(cnt);
            } else if (cnt < 4) {
                // 2~4ms: 맥슨 보간 송신
                send_task_1ms(cnt);
            } else {
                // 5ms: TMotor + 맥슨값 동시 송신
                send_task_5ms();
                cnt = -1;               // 다음 루프에서 0이 됨
                if (ctx.robot_state.load() == RobotState::SHUTTINGDOWN && control_queue.empty()) {
                    ctx.running = false;
                    break;}
            }
            cnt++;

            std::this_thread::sleep_until(next);
        }
    }

    // 고장으로 나왔으면 팔의 위치 유지를 해제한다.
    //
    // 송신만 멈추면 MIT 는 마지막 PD 명령을 계속 걸고 있는다 — 토크 초과로 트립했는데
    // 그 토크를 만들던 명령을 그대로 유지하게 된다. recv_loop 도 같이 죽어 그 뒤로는
    // 감시도 없다. Maxon 은 safety_check_recv_maxon 이 Shutdown 을 보내 꺼지는데
    // TMotor 만 안 꺼지고 있었다.
    //
    // 여기가 send_loop 의 출구라 뒤에 프레임을 보낼 코드가 없다. 트립 시점과 이 시점
    // 사이에 정상 명령이 한 장 더 나갔더라도 감쇠 명령이 버스의 마지막 프레임이 된다.
    //
    // 정상 종료(SHUTTINGDOWN)는 제외한다 — shutdown 포즈에 도착한 상태다.
    // 이 조건의 한계: 종료 중에 발생한 고장은 걸러진다. 실패 방향이 "안 건다" 쪽이라
    // 기존 동작과 같아지므로 허용한다.
    if (ctx.robot_state.load() != RobotState::SHUTTINGDOWN) damped_stop_all();

    ctx.running = false;
    std::cout << "[Controller] send_loop 종료\n";
}

void Controller::recv_loop() {
    while (ctx.running.load()) {
        if (!ctx.recv_active.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // recv_active 전까지 대기
            continue;
        }

        robot.can.clearReadBuffers();

        auto next = std::chrono::steady_clock::now();

        while (ctx.running.load() && ctx.recv_active.load()) {
            next += std::chrono::microseconds(100);     // 100us 주기

            read_frames();
            distribute_frames();

            std::this_thread::sleep_until(next);
        }
    }

    ctx.running = false;
    std::cout << "[Controller] recv_loop 종료\n";
}

// ===== SEND =====
bool Controller::all_tmotors_received() {
    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        if (!tmotor->first_recv_done) return false;
    }
    return true;
}

// NaN/Inf 는 노이즈가 아니라 상류 계산이 깨진 결과다(0/0 등). 송신 전 가드의
// 비교는 NaN 을 전부 통과시키고(abs(NaN) > lim 이 false), 코덱의 int 캐스트는
// INT_MIN 을 만든다. 거부해도 다음 틱에 또 나오므로 호출부가 정지시킨다.
bool Controller::targets_finite(const ControlSetPoint &point) {
    for (int i = 0; i < ROBOT::NUM_JOINT; ++i) {
        if (!std::isfinite(point.q[i])) {
            std::cerr << "[Controller] 목표에 NaN/Inf (관절 " << i << ") — 정지\n";
            return false;
        }
    }
    return true;
}

// MIT 제어 모드 진입. 진입 자체는 통전시키지 않는다 — 첫 command 프레임에서
// kp/kd 가 실리면서 토크가 생긴다. 그래서 게인 0 명령으로 피드백만 유도한다.

// 정책이 소유한 팔 0~8을 슬롯의 최신값으로 채운다.
//
// 슬롯이 비었거나 낡았으면 아무것도 하지 않는 것이 정답이다 — planner가 넣어 둔
// ControlMode::NONE 이 그대로 남아 프레임이 나가지 않고 모터가 직전 명령을 유지한다.
// 기존 안전 가드(POS_DIFF_LIMIT, min/max, 전류)는 그대로 걸린다.
void Controller::merge_policy_target(uint64_t t) {
    PolicyTarget::Snapshot pt = policy_target.snapshot();

    if (!pt.valid || (t - pt.tick) > WATCHDOG_TICKS) {
        // 정책이 이미 반납했으면 잔여 카드를 소비하는 정상 종료다 (큐에 최대 19장 = 95ms,
        // 워치독은 45ms 라 반드시 걸린다). 고장이 아니므로 abort 하지 않는다.
        if (ctx.policy_active.load() && !ctx.policy_fault.exchange(true)) {
            std::cerr << "[Controller] 정책 워치독 — 슬롯이 비었거나 "
                      << (pt.valid ? (t - pt.tick) : 0) << "틱 낡음. 팔 홀드\n";
            ctx.play_abort = true;
        }
        // 팔은 홀드하되 머리는 보정한다. 그냥 return 하면 raw yaw 가 무가드
        // Dynamixel 로 나간다. 팔이 홀드 중이라 낡은 허리각이 곧 현재 명령각이다.
        if (pt.valid) last_waist_for_yaw = pt.q[JointID::WAIST];
        curr_point.q[JointID::HEAD_YAW] = raw_head_yaw - last_waist_for_yaw;

        // merge 가 못 채운 칸을 직전 카드 값으로 메운다. push_setpoint 가 넣은 0 을
        // 그대로 두면 이 카드가 prev_point 가 될 때 다음 카드와의 1ms 보간이 0 에서
        // 출발해 손목(CSP)이 큰 스텝을 받고, 팔 가드도 0 으로 오판한다.
        for (int j = 0; j < JointID::NUM_ARM; ++j) {
            if (curr_point.mode[j] == ControlMode::NONE) curr_point.q[j] = prev_point.q[j];
        }
        return;
    }

    for (int j = 0; j < JointID::NUM_ARM; ++j) {
        curr_point.q[j]    = pt.q[j];
        curr_point.qd[j]   = 0.0;                    // MIT v_des = 0 (함정 3)
        curr_point.mode[j] = (j < 7) ? ControlMode::MIT     // 팔 TMotor
                                     : ControlMode::CST;    // 손목 Maxon
    }

    // 머리 yaw는 planner가 raw로 내보냈다. 허리 보정을 여기서 한다 (함정 2).
    // 대입이어야 한다: 언더플로 틱에는 curr_point 가 재사용되어 merge 가 같은 카드에
    // 다시 돈다. 감산(-=)이면 그때마다 waist 만큼 더 깎여 드리프트한다 (비멱등).
    last_waist_for_yaw = pt.q[JointID::WAIST];
    curr_point.q[JointID::HEAD_YAW] = raw_head_yaw - last_waist_for_yaw;
}

// 모든 TMotor 를 MIT 제어 모드로 진입시킨다.
void Controller::enter_mit_control_mode() {
    struct can_frame frame;

    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        mit_codec.encodeEnterControlMode(tmotor->node_id, &frame);
        robot.can.sendFrame(tmotor->socket, frame);
    }
    std::cout << "[Controller] MIT 제어 모드 진입\n";
}

// kp = kd = t_ff = 0 인 command. 토크를 만들지 않으면서 피드백 프레임만 받아온다.
// p_des는 인코딩상 필요할 뿐 게인이 0이라 아무 영향이 없다.
void Controller::prime_mit_feedback() {
    struct can_frame frame;

    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
            static_cast<float>(tmotor->current_position),
            0.0f, 0.0f, 0.0f, 0.0f,     // kp = kd = t_ff = 0  -> 토크 0
            tmotor->mit_limits());
        robot.can.sendFrame(tmotor->socket, frame);
    }
}

// 안전 트립 시 팔의 위치 유지를 버리고 감쇠만 남긴다.
//
// kp=0 이 핵심이다. 가드 트립(tmotor_send_task)은 kp 를 살려 두는데, 거기는 루프가
// 살아서 매 5ms p_des 를 실측으로 갱신하므로 kp 항이 커질 틈이 없다. 비상정지는
// 1회 보내고 루프가 죽으므로 p_des 가 그 자리에 얼어붙는다 — kp 를 남기면 그게
// 곧 영구 위치 유지가 되어, 트립을 만든 토크를 계속 거는 셈이 된다.
//
// 감쇠는 멈춤이 아니다. 중력이 걸리는 어깨·팔꿈치는 tau=-kd*v 가 중력토크와
// 균형 잡히는 속도(v = tau_grav / kd)로 계속 내려간다. kd 상한이 5 라 더 낮출 수도
// 없다. 허리(수직축)만 실제로 그 자리에 선다.
void Controller::damped_stop_all() {
    struct can_frame frame;

    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        // 엔코더 폴트(code 5)면 감쇠도 포기한다 — tau = -kd*v 의 v 가 그 고장난
        // 엔코더에서 온다. 못 믿을 값으로 토크를 만드느니 0 이 낫다.
        const double kd = (tmotor->current_error == 5) ? 0.0 : tmotor->mit_kd;

        mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
            static_cast<float>(tmotor->current_position),
            0.0f,
            0.0f,                                   // kp = 0 — 위치를 붙잡지 않는다
            static_cast<float>(kd),
            0.0f, tmotor->mit_limits());
        robot.can.sendFrame(tmotor->socket, frame);
    }

    std::cerr << "[Controller] 감쇠 정지 — 팔 위치 유지를 해제했습니다\n";
}


void Controller::send_task_1ms(int cnt) {
    double alpha = static_cast<double>(cnt + 1) / 5.0;

    ControlSetPoint interp;
    interp.mode = curr_point.mode;
    // policy_owns_arm 을 함께 복사한다. maxon_motor_send_task -> cal_torque 가
    // 이 플래그로 중력 보상 여부를 판단하는데, 안 넘기면 기본값 false 가 되어
    // 정책 연주 중 보간 프레임(cnt 0~3, 매 5ms 의 4/5)에 보상이 잘못 켜진다.
    interp.policy_owns_arm = curr_point.policy_owns_arm;
    for (size_t i = 0; i < curr_point.q.size(); ++i) {
        interp.q[i] = prev_point.q[i] + alpha * (curr_point.q[i] - prev_point.q[i]);
    }
 
    maxon_motor_send_task(interp);

    // Sync 프레임: 소켓당 1회
    struct can_frame sync_frame;
    m_codec.encodeSync(&sync_frame);
    for (auto &maxon : robot.virtual_maxon_motor) {
        robot.can.sendFrame(maxon->socket, sync_frame);
    }
}

void Controller::send_task_5ms() {
    tmotor_send_task(curr_point);
    maxon_motor_send_task(curr_point);
    dynamixel_send_task(curr_point);
 
    // Sync 프레임: 소켓당 1회
    struct can_frame sync_frame;
    m_codec.encodeSync(&sync_frame);
    for (auto &maxon : robot.virtual_maxon_motor) {
        robot.can.sendFrame(maxon->socket, sync_frame);
    }
}

void Controller::tmotor_send_task(const ControlSetPoint &point) {
    struct can_frame frame;
 
    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        ControlMode mode = point.mode[id];
        double motor_position  = tmotor->joint_angle_to_motor_position(
            std::clamp(point.q[id], tmotor->min_angle, tmotor->max_angle));
        double motor_velocity = tmotor->direction_sign * point.qd[id];    // rad/s
 
        // 토크 미인가 구간 (START 이전). 게인 0 명령만 보내 피드백을 유지한다.
        // 목표는 실측으로 두고 안전 체크도 건너뛴다 — 오차 자체가 의미 없다.

        // point.mode 가 아니라 tmotor_mit 로 판단한다: ControlSetPoint 의 기본 mode 는
        // POS 이므로, mode 로 걸면 START 이전의 기본 setpoint 가 서보 위치 명령으로
        // 나가 버린다.
        if (ctx.tmotor_mit.load() && !point.torque_on) {
            mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
                static_cast<float>(tmotor->current_position),
                0.0f, 0.0f, 0.0f, 0.0f, tmotor->mit_limits());
            robot.can.sendFrame(tmotor->socket, frame);
            continue;
        }

        // NONE = 정책이 이 관절을 소유한다. 이 카드에는 목표가 없으므로 아무것도 보내지
        // 않는다 (MIT 는 직전 PD 명령을 유지한다). 가드보다 앞이어야 한다 — 뒤에 두면
        // 가드가 목표 없는 값으로 판정하고 트립 프레임까지 보낸다.
        if (mode == ControlMode::NONE) continue;

        // 목표값 안전 체크 (전송 전)
        double desired_joint = std::clamp(point.q[id], tmotor->min_angle, tmotor->max_angle);
        double diff = desired_joint - tmotor->current_joint_angle;
        if (std::abs(diff) > POS_DIFF_LIMIT) {
            // 5ms 주기이므로 1초에 한 번. 매 틱 찍으면 초당 200줄이 되어 다른 로그를 덮는다
            if (tmotor->guard_cnt++ % 200 == 0) {
                std::cerr << "[Controller] TMotor 급변 차단 (" << tmotor->name << ")"
                          << "  desired=" << desired_joint * 180.0 / M_PI << "deg"
                          << "  actual=" << tmotor->current_joint_angle * 180.0 / M_PI << "deg"
                          << "  diff=" << diff * 180.0 / M_PI << "deg"
                          << "  (연속 " << tmotor->guard_cnt << "틱)\n";
            }
            if (ctx.tmotor_mit.load()) {
                // 트립 = 치명. recv 트립과 같은 출구로 나가 damped_stop_all 이 마지막 프레임을 보낸다.
                ctx.running = false;
            }
            continue;
        }
        if (desired_joint < tmotor->min_angle || desired_joint > tmotor->max_angle) {
            if (tmotor->guard_cnt++ % 200 == 0) {
                std::cerr << "[Controller] TMotor 범위 초과 차단 (" << tmotor->name << ")"
                          << "  target=" << desired_joint * 180.0 / M_PI << "deg"
                          << "  (연속 " << tmotor->guard_cnt << "틱)\n";
            }
            if (ctx.tmotor_mit.load()) {
                // 궤적 목표는 그대로 거부하고, 실측을 목표로 보내 그 자리에 버티게 한다.
                // p_des = 실측이므로 tau = Kd(0 - qd) — 순수 감쇠다. 거부한 명령보다
                // 토크가 작고, 피드백이 살아 있어 다음 틱에 새 값으로 재판정한다.
                mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
                    static_cast<float>(tmotor->current_position),
                    0.0f,
                    static_cast<float>(tmotor->mit_kp),
                    static_cast<float>(tmotor->mit_kd),
                    0.0f, tmotor->mit_limits());
                robot.can.sendFrame(tmotor->socket, frame);
            }
            continue;
        }
 
        tmotor->guard_cnt = 0;      // 가드를 통과했다 — 연속 카운트 리셋

        if (mode == ControlMode::MIT) {
            // 게인은 고정이고 목표는 궤적값 그대로다.
            //
            // 게인 램프를 쓰지 않는다. 시작 시점의 위치 오차는 궤적이 실측에서
            // 출발하게 해서(TrajectoryGenerator::generate_trajectory 참조) 애초에
            // 만들지 않는다. 램프는 그 오차를 늦게 드러내기만 했고, 램프가 끝나는
            // 순간 최대 게인 x 최대 오차가 되는 절벽을 만들었다.
            // 게인을 낮추면 그 구간에 중력에 밀리는 부작용도 있었다.
            double p_des = motor_position;

            mit_codec.encodeCommand(&frame, tmotor->node_id, 8,
                static_cast<float>(p_des),
                0.0f,                                       // v_des = 0 (학습의 Kd는 순수 감쇠항)
                static_cast<float>(tmotor->mit_kp),
                static_cast<float>(tmotor->mit_kd),
                0.0f,                                       // t_ff = 0
                tmotor->mit_limits());

            std::vector<double> values = {(double)tmotor->id,
                3.0,    // MIT mode
                p_des,
                tmotor->current_position,
                p_des - tmotor->current_position,
                tmotor->current_torque_mit,     // MIT 는 전류가 아니라 토크
                tmotor->mit_kp                  // 걸린 게인 (이전에는 램프 배율)
            };
            // 속도(모터 보고값 [rad/s], direction_sign 적용 전)와 tick 은 motors_ext 에만 간다.
            log_motor_row(values, tmotor->current_velocity);
        } else if (mode == ControlMode::POS) {
            t_codec.encodePosition(tmotor->node_id, &frame, static_cast<float>(motor_position));

            std::vector<double> values = {(double)tmotor->id,
                1.0,    // position mode
                motor_position,
                tmotor->current_position,
                motor_position - tmotor->current_position,
                tmotor->current_motor_current
            };
            log_motor_row(values);
        } else if (mode == ControlMode::VEL) {
            double err = motor_position - tmotor->current_position;
            double ermp = motor_velocity * tmotor->pole * tmotor->gear_ratio * 60.0 / 2.0 / M_PI;

            double control_input = ermp + tmotor->control_gain * err;
            control_input = std::clamp(control_input, -100000.0, 100000.0);

            t_codec.encodeVelocity(tmotor->node_id, &frame, static_cast<float>(control_input));

            std::vector<double> values = {(double)tmotor->id,
                2.0,    // velocity mode
                motor_position,
                tmotor->current_position,
                motor_position - tmotor->current_position,
                tmotor->current_motor_current,
                control_input
            };
            log_motor_row(values);
        }

        robot.can.sendFrame(tmotor->socket, frame);
    }
}

void Controller::maxon_motor_send_task(const ControlSetPoint &point) {
    struct can_frame frame;
 
    for (auto &[id, motor] : robot.motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;
 
        ControlMode mode = point.mode[id];
        double motor_position  = maxon->joint_angle_to_motor_position(point.q[id]);
 
        if (mode == ControlMode::CSP) {
            if (mode != maxon->mode) set_maxon_mode(maxon, ControlMode::CSP);   // 모터가 현재 설정된 모드와 다르면 변경

            m_codec.encodePosition(maxon->tx_pdo_ids[1], &frame, motor_position);

            std::vector<double> values = {(double)maxon->id,
                1.0,    // CSP mode
                motor_position,
                maxon->current_position,
                motor_position - maxon->current_position,
                maxon->current_torque
            };
            log_motor_row(values);
        } else if (mode == ControlMode::CST) {
            if (mode != maxon->mode) set_maxon_mode(maxon, ControlMode::CST);   // 모터가 현재 설정된 모드와 다르면 변경

            double torque_mNm = cal_torque(maxon, motor_position, point.policy_owns_arm);
            m_codec.encodeTorque(maxon->tx_pdo_ids[3], &frame, static_cast<int>(torque_mNm));

            std::vector<double> values = {(double)maxon->id,
                0.0,    // CST mode
                motor_position,
                maxon->current_position,
                motor_position - maxon->current_position,
                maxon->current_torque,
                torque_mNm
            };
            log_motor_row(values);              // Maxon 은 속도 피드백 없음 -> ext 의 velocity 는 0
        } else if (mode == ControlMode::CSV) {
            std::cerr << "[Controller] MaxonMotor CSV 모드 구현 안됨 (" << maxon->name << ")\n";
            continue;
        } else if (mode == ControlMode::NONE) {
            continue;       // 정책이 손목을 소유한다. 정상이므로 조용히 넘어간다
        }
 
        robot.can.sendFrame(maxon->socket, frame);
    }
}

void Controller::set_maxon_mode(std::shared_ptr<MaxonMotor> &maxon, ControlMode target_mode) {
    struct can_frame frame;

    if (target_mode == ControlMode::CST) {
        m_codec.encodeCSTMode(maxon->can_send_id, &frame);
        robot.can.sendFrame(maxon->socket, frame);
    } else if (target_mode == ControlMode::CSP) {
        m_codec.encodeCSPMode(maxon->can_send_id, &frame);
        robot.can.sendFrame(maxon->socket, frame);
    }

    // 모드 바꾸고 shutdown -> enable 해주기
    m_codec.encodeShutdown(maxon->tx_pdo_ids[0], &frame);
    robot.can.sendFrame(maxon->socket, frame);
    m_codec.encodeEnable(maxon->tx_pdo_ids[0], &frame);
    robot.can.sendFrame(maxon->socket, frame);

    // CST 감쇠항의 차분 기준점을 지금 실측으로 다시 잡는다.
    // prev_pos 는 cal_torque(CST 전용)에서만 갱신되는데 CSP 구간에도 손목은 움직인다.
    // 초기화하지 않으면 CST 로 돌아올 때 낡은 값과 차분해 첫 토크가 포화한다.
    maxon->prev_pos = maxon->current_position;
    maxon->prev_qd  = 0.0;

    maxon->mode = target_mode;
}

double Controller::cal_torque(std::shared_ptr<MaxonMotor> &maxon, double target_position,
                              bool policy_owns_arm) {
    const double dt = 0.001;    // 1ms
    const double alpha = 0.2;   // 저역통과 필터 계수
 
    double err = target_position - maxon->current_position;
    double qd_raw = (maxon->current_position - maxon->prev_pos) / dt;   // 실측을 미분
    double qd_filtered = alpha * qd_raw + (1.0 - alpha) * maxon->prev_qd;
    maxon->prev_pos = maxon->current_position;
    maxon->prev_qd  = qd_filtered;
    double torque_mNm = maxon->control_kp * err - maxon->control_kd * qd_filtered;   // 부호 -

    // 중력 보상.
    // 정책 구간에서는 반드시 꺼야 한다. 학습 제어 법칙에는 보상항이 없고,
    // 시뮬 물리의 중력 처짐을 정책이 이미 감안한 q_des를 내도록 학습됐다.
    // 실기에서 켜면 이중 보상이 되어 스틱이 위로 뜬다.
    //
    // 판단은 setpoint 라벨로 한다. ctx.policy_active 를 여기서 직접 읽으면 곡 종료
    // (release) 후 큐에 남은 정책 소유 CST 프레임(<=100ms)에 보상이 켜져 손목이 뜬다 —
    // merge 와 같은 기준(카드가 만들어질 때의 진실)으로 통일. CSP 는 이 함수를 타지
    // 않으므로 무관하고, CST 개루프/전이(비정책)는 그대로 보상이 걸린다.
    if (policy_owns_arm) {
        return torque_mNm;
    }

    double gravity_angle = 0.0;
    for (auto &[other_id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        bool is_right_wrist = (maxon->name == "right_wrist") &&
                              (tmotor->name == "right_shoulder_2" || tmotor->name == "right_elbow");
        bool is_left_wrist  = (maxon->name == "left_wrist") &&
                              (tmotor->name == "left_shoulder_2"  || tmotor->name == "left_elbow");

        if (is_right_wrist || is_left_wrist) {
            gravity_angle += tmotor->current_joint_angle;
        }
    }
    gravity_angle += maxon->current_joint_angle;

    double gravity_torque_Nm = STICK_MASS_KG * 9.81 * STICK_LEN_M * std::sin(gravity_angle) / maxon->gear_ratio;
    torque_mNm -= gravity_torque_Nm * 1000.0;  // N·m -> mN·m

    return torque_mNm;
}

void Controller::dynamixel_send_task(const ControlSetPoint &point) {
    if (!robot.dxl_sync_write || !robot.dxl_sync_read) return;

    // 모든 dxl 모터의 목표값을 dxl_sync_write에 한 번에 등록
    for (auto &[id, motor] : robot.motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (!dxl) continue;

        // 관절 범위 가드. yaw = raw - waist 라 ±90도를 넘을 수 있는데,
        // angle_to_tick 의 ±180도 클램프는 틱 산술 보호일 뿐 관절 한계가 아니다.
        double joint_angle = std::clamp(point.q[id], dxl->min_angle, dxl->max_angle);
        double motor_position = dxl->joint_angle_to_motor_position(joint_angle);

        int32_t values[3];
        uint8_t param[12];
        // Profile Acceleration, Velocity, Goal Position
        values[0] = 0;  // ms
        values[1] = 0;  // ms
        values[2] = dxl->angle_to_tick(motor_position);
        memcpy(param, values, sizeof(values));

        if (!robot.dxl_sync_write->addParam(dxl->dxl_id, param)) {
            std::cerr << "[Controller] dxl_sync_write addParam failed for ID:"
                    << (int)dxl->dxl_id << "\n";
            continue;
        }
    }

    // 한 번에 송신
    robot.dxl_sync_write->txPacket();
    robot.dxl_sync_write->clearParam();

    // 한 번에 수신
    int comm = robot.dxl_sync_read->txRxPacket();
    if (comm == COMM_SUCCESS) {
        for (auto &[id, motor] : robot.motors) {
            auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
            if (!dxl) continue;

            if (robot.dxl_sync_read->isAvailable(dxl->dxl_id, 132, 4)) {
                int32_t tick = robot.dxl_sync_read->getData(dxl->dxl_id, 132, 4);
                double pos   = dxl->tick_to_angle(tick);
                dxl->current_position    = pos;
                dxl->current_joint_angle = dxl->motor_position_to_joint_angle(pos);
                dxl->first_recv_done     = true;   // release 의 머리 동기화가 실측 유무를 판단하는 근거
            }
        }
    }

    // 로깅
    for (auto &[id, motor] : robot.motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (!dxl) continue;
        // 송신과 같은 값을 남긴다 — 자르기 전 값을 기록하면 로그와 실제가 어긋난다.
        double joint_angle = std::clamp(point.q[id], dxl->min_angle, dxl->max_angle);
        double motor_position = dxl->joint_angle_to_motor_position(joint_angle);
        std::vector<double> values = {
            (double)dxl->id,
            0.0,    // 모드 없음
            motor_position,
            dxl->current_position,
            motor_position - dxl->current_position
        };
        log_motor_row(values);
    }
}

// ===== RECV =====
void Controller::read_frames() {
    struct can_frame frame;

    for (auto &[ifname, socket_fd] : robot.can.getSocket()) {
        while (true) {
            ssize_t bytes = read(socket_fd, &frame, sizeof(frame));
            if (bytes == sizeof(frame)) {
                temp_frames[socket_fd].push_back(frame);
            } else if (bytes < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
                break;  // 더 읽을 프레임 없음
            } else {
                break;  // 기타 오류
            }
        }
    }
}

void Controller::distribute_frames() {
    bool is_safe = true;
    bool any_update = false;

    for (auto &[id, motor_ptr] : robot.motors) {
        if (auto tmotor = std::dynamic_pointer_cast<TMotor>(motor_ptr)) {
            const bool mit = ctx.tmotor_mit.load();

            for (auto &frame : temp_frames[tmotor->socket]) {
                // 서보는 can_id 하위 바이트가 모터 id지만, MIT는 can_id가 0으로 오고
                // 식별자가 data[0]에 들어온다. 매칭 규칙 자체가 다르다.
                const bool match = mit ? (frame.data[0] == tmotor->node_id)
                                       : ((frame.can_id & 0xFF) == tmotor->node_id);
                if (!match) continue;

                if (mit) {
                    auto [mid, pos, spd, torque, merr] = mit_codec.decodeFeedback(&frame, tmotor->mit_limits());
                    tmotor->current_error = merr;
                    tmotor->current_position    = pos;
                    tmotor->current_velocity    = spd;
                    tmotor->current_torque_mit  = torque;
                } else {
                    auto [mid, pos, spd, cur, temp, err] = t_codec.decodeFeedback(&frame);
                    tmotor->current_position      = pos;
                    // spd 는 ERPM 이다. current_velocity 는 항상 rad/s 로 유지한다 —
                    // 안 그러면 publish_joint_snapshot 이 정책 obs 에 2000배 값을 넣는다.
                    // encodeVelocity 의 역: rad/s = ERPM x 2pi / (60 x pole x gear)
                    const double erpm_to_rad_s =
                        2.0 * M_PI / (60.0 * tmotor->pole * tmotor->gear_ratio);
                    tmotor->current_velocity      = spd * erpm_to_rad_s;
                    tmotor->current_motor_current = cur;
                }
                tmotor->current_joint_angle = tmotor->motor_position_to_joint_angle(tmotor->current_position);

                if (!safety_check_recv_tmotor(tmotor)) {
                    is_safe = false;
                }

                tmotor->first_recv_done = true; // 수신 확인 완료되면 send loop 켜기
                any_update = true;
            }
        } else if (auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor_ptr)) {
            for (auto &frame : temp_frames[maxon->socket]) {
                if (frame.can_id == maxon->rx_pdo_ids[0]) {
                    auto [mid, pos, torque, status] = m_codec.decodeFeedback(&frame);
                    maxon->current_position    = pos;
                    maxon->current_torque      = torque;
                    maxon->status_bit          = status;
                    maxon->current_joint_angle = maxon->motor_position_to_joint_angle(pos);
                    maxon->first_recv_done     = true;

                    if (!safety_check_recv_maxon(maxon)) {
                        is_safe = false;
                    }
                }
            }
        }
    }

    temp_frames.clear();

    // recv는 100us마다 돌지만 프레임은 그보다 드물게 온다. 갱신이 없으면 발행하지 않는다.
    if (any_update) publish_joint_snapshot();

    if (!is_safe) {
        ctx.running = false;
    }
}

// 팔 9관절을 한 번에 발행한다. 정책이 모터를 하나씩 순회해 읽으면
// 100us 주기의 recv가 중간을 갱신해 "관절 0은 t, 관절 5는 t+50us"인
// 뒤섞인 상태가 obs로 들어간다 (함정 4).
void Controller::publish_joint_snapshot() {
    JointSnapshot::Data d;

    for (int j = 0; j < JointID::NUM_ARM; ++j) {
        auto it = robot.motors.find(j);
        if (it == robot.motors.end()) continue;

        d.q[j] = it->second->current_joint_angle;

        // TMotor는 피드백에 속도가 있다. 손목(7,8)은 Maxon이라 없으므로 0으로 두고
        // PolicyRunner가 t_pub_ns 간격으로 유한차분해 채운다 — 100us 차분은
        // 엔코더 양자화 노이즈가 커서 쓸 수 없다.
        if (auto tmotor = std::dynamic_pointer_cast<TMotor>(it->second)) {
            d.qd[j] = tmotor->direction_sign * tmotor->current_velocity;
        }
    }

    d.t_pub_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    d.valid = true;

    robot.joint_snapshot.publish(d);
}

bool Controller::safety_check_recv_tmotor(std::shared_ptr<TMotor> &motor) {
    double angle = motor->current_joint_angle;

    if (angle < motor->min_angle || angle > motor->max_angle) {
        std::cerr << "[Controller] TMotor 범위 초과 (" << motor->name << ")"
                  << "  joint=" << angle * 180.0 / M_PI << "deg\n";
        return false;
    }

    // 모터가 보고한 에러 코드. 0 이 아니면 정상 상태가 아니므로 멈춘다.
    if (ctx.tmotor_mit.load() && motor->current_error != 0) {
        static const char* kErr[8] = {"정상", "모터 과열", "과전류", "과전압",
                                      "저전압", "엔코더 폴트", "MOSFET 과열", "모터 스톨"};
        const int e = motor->current_error;
        std::cerr << "[Controller] TMotor 에러 (" << motor->name << ")"
                  << "  code=" << e << " ("
                  << (e >= 0 && e < 8 ? kErr[e] : "알 수 없음") << ")\n";
        return false;
    }

    // 과부하 차단 — 서보 원본 로직을 MIT 에 복원 (연속 5프레임 = 25ms 초과 -> 정지).
    if (ctx.tmotor_mit.load()) {
        if (std::abs(motor->current_torque_mit) > motor->mit_torque_safety) {
            if (motor->cnt++ > 0) {
                std::cerr << "[Controller] TMotor 토크 초과 (" << motor->name << ")"
                          << "  torque=" << motor->current_torque_mit << "Nm"
                          << "  limit=" << motor->mit_torque_safety << "Nm\n";
                return false;
            }
        } else {
            motor->cnt = 0;
        }
        return true;
    }

    // 서보 모드는 기존 동작 유지 (연속 5회 초과 -> 정지)
    if (motor->current_motor_current > motor->current_limit) {
        if (motor->cnt++ > 0) {
            std::cerr << "[Controller] TMotor 전류 초과 (" << motor->name << ")"
                      << "  current=" << motor->current_motor_current << "A\n";
            return false;
        }
    } else {
        motor->cnt = 0;
    }

    return true;
}

bool Controller::safety_check_recv_maxon(std::shared_ptr<MaxonMotor> &motor) {
    double angle = motor->current_joint_angle;

    if (angle < motor->min_angle || angle > motor->max_angle) {
        std::cerr << "[Controller] MaxonMotor 범위 초과 (" << motor->name << ")"
                  << "  joint=" << angle * 180.0 / M_PI << "deg\n";

        // encodeShutdown 송신
        struct can_frame frame;
        m_codec.encodeShutdown(motor->tx_pdo_ids[0], &frame);
        robot.can.sendFrame(motor->socket, frame);

        struct can_frame sync_frame;
        m_codec.encodeSync(&sync_frame);
        robot.can.sendFrame(motor->socket, sync_frame);

        return false;
    }
    return true;
}
