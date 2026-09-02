#pragma once

#include <queue>
#include <mutex>
#include <vector>
#include <array>
#include <optional>

#include "common/robot_config.hpp"

enum class ControlMode {
    // T motor
    POS,
    VEL,
    MIT,        // MIT 모드: tau = Kp(p_des - p) + Kd(v_des - qd) + t_ff
    
    // Maxon Motor
    CST,
    CSV,
    CSP,

    // Dynamixel or None
    NONE,
};

struct ControlSetPoint {
    std::array<double, ROBOT::NUM_JOINT> q{};
    std::array<double, ROBOT::NUM_JOINT> qd{};
    std::array<ControlMode, ROBOT::NUM_JOINT> mode{};

    bool audio_start = false;

    // 이 setpoint를 만들 때 팔 0~8이 정책 소유였는지.
    //
    // ctx.policy_active 를 직접 보면 안 되는 이유: 궤적은 100ms 앞서 생성되고
    // send_loop 은 그걸 나중에 소비한다. 두 시점에 플래그를 따로 읽으면 경계에서
    // 어긋난다 — 특히 "생성 때는 정책이었는데 소비 때는 아님" 이면 팔에 q=0 이
    // 그대로 나간다. 소유권을 데이터에 실어 보내면 구조적으로 어긋날 수 없다.
    bool policy_owns_arm = false;

    // 이 setpoint 를 만들 때 TMotor 토크가 인가된 상태였는지.
    //
    // false 면 send_loop 이 MIT 명령을 게인 0 으로 보낸다 — 프레임은 나가므로
    // 피드백은 계속 받지만 토크는 0 이다. START 로 궤적이 처음 생성될 때 true 가
    // 된다. policy_owns_arm 과 같은 이유로 데이터에 실어 보낸다: 궤적 생성은
    // 소비보다 앞서므로, ctx 의 플래그를 소비 시점에 읽으면 아직 큐에 남아 있는
    // START 이전 목표(init 자세)에 최대 게인이 걸린다.
    bool torque_on = false;

    // 이 setpoint의 악보 시간 [s]. send_loop이 pop할 때 ctx.t_score로 발행한다.
    // 정책이 tick x 5ms로 시간을 따로 계산하면 배속 재생에서 팔과 발이 어긋난다 —
    // get_num_point()의 round_sum 반올림 보정과 play_speed_scale이 여기 이미 반영돼 있다.
    double t_score = 0.0;
};

class ControlQueue {
public:
    ControlQueue();
    ~ControlQueue();

    void push(const ControlSetPoint& point);
    bool empty();
    size_t size();
    std::optional<ControlSetPoint> try_pop();

private:
    std::queue<ControlSetPoint> queue_;
    std::mutex mutex_;
};