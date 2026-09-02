#include <thread>
#include "trajectory/trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator(AppContext& ctxRef, ControlQueue &controlQueueRef, Robot& robotRef)
    : ctx(ctxRef), control_queue(controlQueueRef), robot(robotRef),
      play_motion_generator(ctxRef), trajectory_log("trajectory") {
    
    std::vector<std::string> header = {
        "joint 0", "joint 1", "joint 2", "joint 3", "joint 4",
        "joint 5", "joint 6", "joint 7", "joint 8",
        "joint 9", "joint 10", "joint 11", "joint 12"
    };
    trajectory_log.set_header(header);
}

TrajectoryGenerator::~TrajectoryGenerator() {

}

void TrajectoryGenerator::initialize(const std::map<std::string, std::vector<double>>& pose) {
    // 팔 TMotor 제어 모드. motors.json 최상위 "tmotor_mit"으로 결정된다.
    tmotor_control_mode = ctx.tmotor_mit.load() ? ControlMode::MIT : ControlMode::VEL;

    solver.initialize();
    play_motion_generator.initialize();

    update_last_q(pose.at("init"));
    ready_pose = pose.at("ready");
}

void TrajectoryGenerator::generate_trajectory(const MotionPrimitive& motion) {
    // 토크 인가 직전에 궤적 출발점을 실측으로 맞춘다.
    //
    // last_q 의 초기값은 init 자세다. init 과 home 이 같은 값이므로 START 궤적은
    // sample(90 -> 90) = 상수가 되어, 목표가 3초 내내 home 에 고정된다.
    // 모터가 그 자리에 없으면 오차가 첫 틱부터 끝까지 유지되고, MIT 는 오차가 곧
    // 토크이므로 즉시 큰 토크가 걸린다 (실측: 28도 어긋난 상태에서 49 N·m 요구).
    //
    // 실측에서 출발시키면 sample() 이 실제로 구간을 나눈다 — 틱당 목표 이동이
    // 0.07도 수준이라 오차가 커질 틈이 없고, PD 는 추종에 필요한 토크만 낸다.
    if (ctx.sync_last_q_requested.exchange(false)) {
        // 실측을 못 읽으면 토크를 켜지 않는다 — 출발점이 틀린 궤적에 게인을 걸면
        // 첫 틱부터 큰 오차가 토크로 나간다.
        if (sync_last_q_from_robot()) torque_on = true;
        else std::cerr << "[TrajectoryGenerator] 토크 미인가 상태로 진행\n";
    }

    switch (motion.type) {
    case MotionType::STANDBY:
        generate_standby_trajectory();  // 키 제거하기 전 현재 위치 유지 (고정)
        break;
    case MotionType::TRANSLATE:
        if (motion.space == TrajectorySpace::JOINT) {
            generate_joint_space_trajectory(motion);
        } else if (motion.space == TrajectorySpace::TASK) {
            generate_task_space_trajectory(motion);
        } else {
            std::cerr << "[TrajectoryGenerator] Unknown motion trajectory space\n";
        }
        break;
    case MotionType::DRUM:
        if (motion.flag == PlayFlag::START) {
            generate_play_start_trajectory(motion);
        } else if (motion.flag == PlayFlag::END) {
            generate_play_end_trajectory();
        } else {
            generate_play_trajectory(motion);
        }
        break;
    case MotionType::IDLE:
        generate_idle_trajectory();
        break;
    default:
        std::cerr << "[TrajectoryGenerator] Unknown motion type\n";
        break;
    }
}

void TrajectoryGenerator::generate_standby_trajectory() {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    double t_total = 1.0;
    int num_point = static_cast<int>(t_total / ROBOT::DT_SECOND);

    std::vector<double> q0(last_q.begin(), last_q.end());
    std::vector<double> q1(last_q.begin(), last_q.end());

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample_cosine(q0, q1, num_point, k);

        ControlSetPoint set_point;
        std::copy(q.begin(),  q.end(),  set_point.q.begin());
        std::copy(qd.begin(), qd.end(), set_point.qd.begin());
        set_point.mode = modes;
        push_setpoint(set_point);

        trajectory_log.record(set_point.q);
    }
}

void TrajectoryGenerator::generate_joint_space_trajectory(const MotionPrimitive& motion) {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    int num_point = static_cast<int>(motion.t_total / ROBOT::DT_SECOND);

    std::vector<double> q0(last_q.begin(), last_q.end());
    std::vector<double> q1 = motion.q_target;

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample(q0, q1, num_point, k, motion.profile);

        ControlSetPoint set_point;
        std::copy(q.begin(),  q.end(),  set_point.q.begin());
        std::copy(qd.begin(), qd.end(), set_point.qd.begin());
        set_point.mode = modes;
        push_setpoint(set_point);

        trajectory_log.record(set_point.q);
    }

    update_last_q(q1);
}

void TrajectoryGenerator::generate_task_space_trajectory(const MotionPrimitive& motion) {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    int num_point = static_cast<int>(motion.t_total / ROBOT::DT_SECOND);

    std::vector<double> q0(last_q.begin(), last_q.end());
    std::vector<double> q1 = motion.q_target;

    std::vector<double> p0 = {
        last_p_R[0], last_p_R[1], last_p_R[2],
        last_p_L[0], last_p_L[1], last_p_L[2]
    };

    std::vector<double> p1 = {
        motion.p_target_R[0], motion.p_target_R[1], motion.p_target_R[2],
        motion.p_target_L[0], motion.p_target_L[1], motion.p_target_L[2]
    };

    // 속도 계산을 위한 이전 관절각
    std::array<double, 9> prev_q;
    std::copy(last_q.begin(), last_q.begin() + 9, prev_q.begin());

    std::queue<ControlSetPoint> buffer;

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample(q0, q1, num_point, k, motion.profile);
        auto [p, pd] = sample(p0, p1, num_point, k, motion.profile);

        std::array<double, 3> pR = {p[0], p[1], p[2]};
        std::array<double, 3> pL = {p[3], p[4], p[5]};
        double theta0 = q[0];
        double theta7 = q[7];
        double theta8 = q[8];
        KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, theta0, theta7, theta8, true);

        if (!result.success) {
            std::cerr << "[TrajectoryGenerator] Failed to solve inverse kinematics\n";
            return;
        }

        // 13차원 set_point 구성: IK 결과(0~8) + 관절 보간값(9~12)
        ControlSetPoint set_point;
        for (int i = 0; i < 9; i++) {
            set_point.q[i] = result.q[i];   // 관절 0~8 (팔)
            set_point.qd[i] = (result.q[i] - prev_q[i]) / ROBOT::DT_SECOND;
        }
        for (int i = 9; i < ROBOT::NUM_JOINT; i++) {
            set_point.q[i] = q[i];          // 관절 9~12 (페달, 머리)
            set_point.qd[i] = qd[i]; 
        }
        set_point.mode = modes;
        buffer.push(set_point);

        prev_q = result.q;
    }

    // IK 오류가 없으면 적재
    while (!buffer.empty()) {
        ControlSetPoint sp = buffer.front();
        buffer.pop();
        push_setpoint(sp);

        trajectory_log.record(sp.q);
    }

    update_last_q(p1, q1);
}

void TrajectoryGenerator::generate_play_start_trajectory(const MotionPrimitive& motion) {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    double t_total = 4.0;
    int num_point = static_cast<int>(t_total / ROBOT::DT_SECOND);

    std::array<double, ROBOT::NUM_JOINT> q_target;
    if (play_motion_generator.reset(q_target, motion.init_note_r, motion.init_note_l)) {
        std::vector<double> q0(last_q.begin(), last_q.end());
        std::vector<double> q1(q_target.begin(), q_target.end());

        for (int k = 1; k <= num_point; k++) {
            auto [q, qd] = sample(q0, q1, num_point, k, TrajectoryProfile::COSINE);

            ControlSetPoint set_point;
            std::copy(q.begin(),  q.end(),  set_point.q.begin());
            std::copy(qd.begin(), qd.end(), set_point.qd.begin());
            set_point.mode = modes;
            push_setpoint(set_point);

            trajectory_log.record(set_point.q);
        }

        update_last_q(q1);

        first_point = true; // 음악 재생을 위함

        // ready 자세까지의 전이 궤적을 모두 밀어 넣은 뒤에 넘긴다.
        // 위 setpoint 들은 policy_owns_arm=false 로 이미 큐에 들어갔으므로
        // 팔이 그 궤적을 끝까지 따라간 다음 정책이 이어받는다.
        acquire_policy();
    } else {
        ctx.play_abort = true;
    }
}

void TrajectoryGenerator::generate_play_end_trajectory() {
    // 궤적을 만들기 전에 되받는다. last_q 가 실측으로 맞춰진 뒤 q0 을 떠야
    // 복귀 궤적이 점프하지 않는다.
    release_policy();

    // play 후 레디 자세로 이동
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    double t_total = 4.0;
    int num_point = static_cast<int>(t_total / ROBOT::DT_SECOND);

    std::vector<double> q0(last_q.begin(), last_q.end());
    std::vector<double> q1 = ready_pose;

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample(q0, q1, num_point, k, TrajectoryProfile::COSINE);

        ControlSetPoint set_point;
        std::copy(q.begin(),  q.end(),  set_point.q.begin());
        std::copy(qd.begin(), qd.end(), set_point.qd.begin());
        set_point.mode = modes;
        push_setpoint(set_point);

        trajectory_log.record(set_point.q);
    }

    update_last_q(q1);
}

void TrajectoryGenerator::generate_play_trajectory(const MotionPrimitive& motion) {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes(true);

    std::queue<std::array<double, ROBOT::NUM_JOINT>> play_motion = play_motion_generator.generate_motion(motion.robotic_drum_score);

    // 속도 계산을 위한 이전 관절각
    std::array<double, ROBOT::NUM_JOINT> prev_q = last_q;

    if (play_motion.empty()) {
        ctx.play_abort = true;
        return;
    }

    while (!play_motion.empty()) {
        std::array<double, ROBOT::NUM_JOINT> q = play_motion.front();
        play_motion.pop();

        ControlSetPoint set_point;
        for (int i = 0; i < ROBOT::NUM_JOINT; i++) {
            set_point.q[i] = q[i];
            set_point.qd[i] = (q[i] - prev_q[i]) / ROBOT::DT_SECOND;    // TODO: 수치 미분 스파이크 무서우면 필터 넣기
        }

        // 싱크 맞춰서 음악 재생
        if (first_point) {
            first_point = false;
            set_point.audio_start = true;
        }
        
        set_point.mode = modes;
        push_setpoint(set_point);

        prev_q = q;

        trajectory_log.record(set_point.q);
    }

    update_last_q(prev_q);
}

void TrajectoryGenerator::generate_idle_trajectory() {
    std::array<ControlMode, ROBOT::NUM_JOINT> modes = get_modes();
    double t_total = 1.0;
    int num_point = static_cast<int>(t_total / ROBOT::DT_SECOND);

    std::vector<double> q0(last_q.begin(), last_q.end());
    std::vector<double> q1(last_q.begin(), last_q.end());    // TODO: 미세한 움직임 구현

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample_cosine(q0, q1, num_point, k);

        ControlSetPoint set_point;
        std::copy(q.begin(),  q.end(),  set_point.q.begin());
        std::copy(qd.begin(), qd.end(), set_point.qd.begin());
        set_point.mode = modes;
        push_setpoint(set_point);

        trajectory_log.record(set_point.q);
    }

    update_last_q(q1);
}

// ControlQueue로 나가는 유일한 통로.
//
// 정책 구간에서는 팔 0~8을 ControlMode::NONE 으로 비워 둔다. NONE이 곧 fail-safe다 —
// tmotor_send_task / maxon_motor_send_task가 NONE을 만나면 프레임을 보내지 않고,
// 프레임 미송신은 모터가 직전 명령을 유지한다는 뜻이다. send_loop의 머지가 어떤 이유로든
// 실패해도(슬롯이 비었거나 낡았거나) 팔은 자동으로 홀드된다.
void TrajectoryGenerator::push_setpoint(ControlSetPoint& sp) {
    // 소유권을 여기서 한 번 읽어 setpoint 에 실어 보낸다. send_loop 은 이 필드만 보고
    // 판단하므로, 생성과 소비 사이에 ctx.policy_active 가 뒤집혀도 어긋나지 않는다.
    sp.policy_owns_arm = ctx.policy_active.load();
    sp.torque_on       = torque_on;

    if (sp.policy_owns_arm) {
        for (int j = 0; j < JointID::NUM_ARM; ++j) {
            sp.q[j]    = 0.0;
            sp.qd[j]   = 0.0;
            sp.mode[j] = ControlMode::NONE;   // "이 관절은 내 소유가 아님"
        }
    }
    sp.t_score = cur_t_score;
    control_queue.push(sp);
}

// 정책에 팔 0~8을 넘긴다.
//
// 거절 조건이 있다. 하나라도 걸리면 정책 없이 기존 개루프 경로로 연주한다 —
// 넘긴 뒤에 문제가 드러나면 팔이 주인 없이 남아 위험하므로, 넘기기 전에 본다.
bool TrajectoryGenerator::acquire_policy() {
    if (!ctx.policy_ready.load()) {
        std::cerr << "[TrajectoryGenerator] 정책 미준비 — 개루프로 연주합니다\n";
        return false;
    }
    if (!ctx.tmotor_mit.load()) {
        // 정책 출력은 위치 목표이고 MIT 의 Kp/Kd 로 추종한다. 서보 모드에는 실을 곳이 없다.
        std::cerr << "[TrajectoryGenerator] TMotor가 MIT 모드가 아님 — 개루프로 연주합니다\n";
        return false;
    }
    for (int j = 0; j < JointID::NUM_ARM; ++j) {
        if (robot.motors.find(j) == robot.motors.end()) {
            std::cerr << "[TrajectoryGenerator] 팔 모터 " << j
                      << " 결번 — 개루프로 연주합니다\n";
            return false;
        }
    }

    ctx.policy_fault = false;
    ctx.policy_epoch.fetch_add(1);   // ObsBuilder reset 유도
    ctx.policy_active = true;
    std::cerr << "[TrajectoryGenerator] 팔 0~8 소유권을 정책에 넘겼습니다\n";
    return true;
}

// 팔을 되받는다. 정책이 움직인 실제 자세에서 복귀 궤적이 출발하도록 last_q 를 맞춘다.
void TrajectoryGenerator::release_policy() {
    if (!ctx.policy_active.exchange(false)) return;
    sync_last_q_from_robot();
    std::cerr << "[TrajectoryGenerator] 팔 0~8 소유권을 되받았습니다\n";
}

// 실측 팔 관절각을 last_q에 되쓴다. 9~12(발·머리)는 planner가 계속 소유하므로 건드리지 않는다.
bool TrajectoryGenerator::sync_last_q_from_robot() {
    // 피드백이 한 번이라도 들어온 뒤에 읽어야 한다. MIT 는 명령에 대한 응답으로만
    // 피드백을 주므로, send_loop 의 예열(게인 0)이 돌 시간을 준다.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    for (;;) {
        bool ready = true;
        for (int j = 0; j < JointID::NUM_ARM && ready; ++j) {
            auto it = robot.motors.find(j);
            if (it == robot.motors.end()) continue;
            auto tm = std::dynamic_pointer_cast<TMotor>(it->second);
            if (tm && !tm->first_recv_done) ready = false;
        }
        if (ready) break;
        if (std::chrono::steady_clock::now() > deadline) {
            std::cerr << "[TrajectoryGenerator] 실측 동기화 실패 — 피드백 없음\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (int j = 0; j < JointID::NUM_ARM; ++j) {
        auto it = robot.motors.find(j);
        if (it == robot.motors.end()) continue;
        last_q[j] = it->second->current_joint_angle;
        last_qd[j] = 0.0;
    }
    std::cerr << "[TrajectoryGenerator] last_q를 실측으로 동기화 (팔 0~8)\n";
    return true;
}

std::array<ControlMode, ROBOT::NUM_JOINT> TrajectoryGenerator::get_modes(bool is_play) {
    if (is_play) {
        wrist_control_mode = ControlMode::CST;
    } else {
        wrist_control_mode = ControlMode::CSP;
    }

    std::array<ControlMode, ROBOT::NUM_JOINT> modes = {
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        wrist_control_mode,
        wrist_control_mode,
        pedal_control_mode,
        pedal_control_mode,
        ControlMode::NONE,
        ControlMode::NONE
    };

    return modes;
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k,
    TrajectoryProfile profile
) {
    std::pair<std::vector<double>, std::vector<double>> result;

    switch (profile) {
    case TrajectoryProfile::TRAPEZOIDAL:
        result = sample_trapezoidal(q0, q1, n, k);
        break;
    case TrajectoryProfile::CUBIC:
        result = sample_cubic(q0, q1, n, k);
        break;
    case TrajectoryProfile::QUINTIC:
        result = sample_quintic(q0, q1, n, k);
        break;
    case TrajectoryProfile::COSINE:
        result = sample_cosine(q0, q1, n, k);
        break;
    }

    return result;
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_trapezoidal(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 사다리꼴 속도 프로파일: 가속(1/4) - 등속(1/2) - 감속(1/4)
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)

    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);   // 정규화 시간 [0, 1)
    double t_total = n * ROBOT::DT_SECOND;                                      // 실제 전체 시간 [s]
 
    double s_pos;   // 정규화 위치 [0, 1]
    double s_vel;   // 정규화 속도 ds/d(s_time)
 
    const double t_acc = 0.25;      // 가속 구간 비율
    const double v_max = 4.0 / 3.0; // 최대 정규화 속도 (등속 구간 속도)
 
    if (s < t_acc) {
        // 가속 구간
        s_pos = 0.5 * v_max * (s * s) / t_acc;
        s_vel = v_max * (s / t_acc);
    } else if (s < 1.0 - t_acc) {
        // 등속 구간
        s_pos = 0.5 * v_max * t_acc + v_max * (s - t_acc);
        s_vel = v_max;
    } else {
        // 감속 구간
        double tau = 1.0 - s;
        s_pos = 1.0 - 0.5 * v_max * (tau * tau) / t_acc;
        s_vel = v_max * (tau / t_acc);
    }
    s_pos = std::clamp(s_pos, 0.0, 1.0);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_cubic(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 3차 다항식 보간: 양 끝점에서 속도 0
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)
 
    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * ROBOT::DT_SECOND;
 
    double s_pos = 3.0 * s * s - 2.0 * s * s * s;
    double s_vel = 6.0 * s * (1.0 - s);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_quintic(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 5차 다항식 보간: 양 끝점에서 속도와 가속도 모두 0
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)

    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * ROBOT::DT_SECOND;
 
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
 
    double s_pos = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    double s_vel = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_cosine(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 사인(코사인) 보간: s(t) = (1 - cos(πt)) / 2
    // 양 끝점에서 속도 0, 매끄러운 가속/감속
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)
 
    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * ROBOT::DT_SECOND;
 
    double s_pos = 0.5 * (1.0 - std::cos(M_PI * s));
    double s_vel = 0.5 * M_PI * std::sin(M_PI * s);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

void TrajectoryGenerator::update_last_q(const std::vector<double>& q) {
    std::copy(q.begin(), q.end(), last_q.begin());
    last_qd.fill(0.0);

    std::array<double, 9> q_in;
    std::copy(last_q.begin(), last_q.begin() + 9, q_in.begin());
    KinematicsSolver::FKResult result = solver.solve_fk(q_in);

    if (!result.success) {
        std::cerr << "[TrajectoryGenerator] Failed to solve forward kinematics\n";
        return;
    }
    
    last_p_R = result.pR;
    last_p_L = result.pL;
}

void TrajectoryGenerator::update_last_q(const std::array<double, ROBOT::NUM_JOINT>& q) {
    last_q = q;
    last_qd.fill(0.0);

    std::array<double, 9> q_in;
    std::copy(last_q.begin(), last_q.begin() + 9, q_in.begin());
    KinematicsSolver::FKResult result = solver.solve_fk(q_in);

    if (!result.success) {
        std::cerr << "[TrajectoryGenerator] Failed to solve forward kinematics\n";
        return;
    }
    
    last_p_R = result.pR;
    last_p_L = result.pL;
}

void TrajectoryGenerator::update_last_q(const std::vector<double>& p, const std::vector<double>& q) {
    std::array<double, 3> pR = {p[0], p[1], p[2]};
    std::array<double, 3> pL = {p[3], p[4], p[5]};
    double theta0 = q[0];
    double theta7 = q[7];
    double theta8 = q[8];
    KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, theta0, theta7, theta8, true);

    if (!result.success) {
        std::cerr << "[TrajectoryGenerator] Failed to solve inverse kinematics\n";
        return;
    }

    // 13차원 last_q 구성: IK 결과(0~8) + 관절 보간값(9~12)
    for (int i = 0; i < 9; i++) {
        last_q[i] = result.q[i];
    }
    for (int i = 9; i < ROBOT::NUM_JOINT; i++) {
        last_q[i] = q[i];
    }
    last_qd.fill(0.0);

    last_p_R = pR;
    last_p_L = pL;
}