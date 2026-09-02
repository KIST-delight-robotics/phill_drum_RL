#pragma once

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>
#include <queue>
#include <array>

#include "common/app_context.hpp"
#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"  // MotionPrimitive
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "trajectory/play_motion_generator.hpp"
#include "hardware/robot.hpp"
#include "util/logger.hpp"

class TrajectoryGenerator {
public:
    TrajectoryGenerator(AppContext& ctxRef, ControlQueue &controlQueueRef, Robot& robotRef);
    ~TrajectoryGenerator();

    void initialize(const std::map<std::string, std::vector<double>>& pose);
    void generate_trajectory(const MotionPrimitive& motion);

    // 정책에 팔을 넘긴다/되받는다. START 궤적 끝과 END 궤적 앞에서 부른다.
    // 넘길 수 없는 상태면(런너 미준비, MIT 아님, 팔 모터 결번) 넘기지 않고 false 를
    // 반환한다 — 그 경우 기존 개루프 경로로 연주한다.
    bool acquire_policy();
    void release_policy();

    // 정책 구간이 끝날 때 실측 팔 관절각을 last_q에 되쓴다.
    // PLAYING 동안 팔은 정책이 움직이는데 TrajectoryGenerator는 그걸 모르므로,
    // 그냥 두면 복귀 궤적이 START 때 멈춘 낡은 값에서 출발해 수십 도 점프한다.
    void sync_last_q_from_robot();
 
private:
    AppContext &ctx;
    ControlQueue &control_queue;
    Robot &robot;

    KinematicsSolver solver;
    PlayMotionGenerator play_motion_generator;

    std::array<double, ROBOT::NUM_JOINT> last_q;     // 마지막 위치
    std::array<double, ROBOT::NUM_JOINT> last_qd;    // 마지막 속도

    std::array<double, 3> last_p_R;   // 마지막 위치
    std::array<double, 3> last_p_L;   // 마지막 위치

    ControlMode tmotor_control_mode = ControlMode::VEL;  // initialize()에서 ctx.tmotor_mit에 따라 MIT로 바뀐다
    ControlMode wrist_control_mode = ControlMode::CSP;
    ControlMode pedal_control_mode = ControlMode::CSP;
    
    void generate_standby_trajectory();
    void generate_joint_space_trajectory(const MotionPrimitive& motion);
    void generate_task_space_trajectory(const MotionPrimitive& motion);
    void generate_play_start_trajectory(const MotionPrimitive& motion);
    void generate_play_end_trajectory();
    void generate_play_trajectory(const MotionPrimitive& motion);
    void generate_idle_trajectory();

    // ControlQueue로 나가는 유일한 통로. 정책 구간이면 팔 0~8의 소유권을 넘긴다.
    void push_setpoint(ControlSetPoint& sp);

    // 현재 생성 중인 구간의 악보 시간. push_setpoint가 setpoint에 실어 보낸다.
    double cur_t_score = 0.0;

    std::vector<double> ready_pose;   // play 후 돌아오는 대기 자세 (다음 동작 준비 상태)

    std::array<ControlMode, ROBOT::NUM_JOINT> get_modes(bool is_play = false);
    std::pair<std::vector<double>, std::vector<double>> sample(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k,
        TrajectoryProfile profile
    );
    std::pair<std::vector<double>, std::vector<double>> sample_trapezoidal(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_cubic(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_quintic(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_cosine(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    void update_last_q(const std::vector<double>& q);
    void update_last_q(const std::array<double, ROBOT::NUM_JOINT>& q);
    void update_last_q(const std::vector<double>& p, const std::vector<double>& q);

    // ===== log =====
    Logger trajectory_log;

    // ===== audio =====
    bool first_point = false;
};