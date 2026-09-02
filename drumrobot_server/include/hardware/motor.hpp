#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <cstring>
#include <utility>

#include "dynamixel_sdk.h"

#include "common/control_queue.hpp"
#include "hardware/motor_codec.hpp"

class Motor{
public:
    Motor(int id);
    virtual ~Motor();

    int id;
    std::string name;

    double direction_sign;
    double initial_joint_angle;
    double min_angle;
    double max_angle;
    double motor_min;
    double motor_max;

    double current_joint_angle = 0.0;

    double joint_angle_to_motor_position(double joint_angle);
    double motor_position_to_joint_angle(double motor_position);
private:
};

class TMotor : public Motor {
public:
    TMotor(int id);

    // For CAN communication
    uint32_t node_id;
    int socket;
    bool is_connected = false;

    std::string model;          // 모델
    // pole *pair* 수 = 21 (총 자극 42개). AK10-9 / AK70-10 동일 — 확인됨.
    //   변수명이 pole 이지만 담는 값은 pole pair 다. 42 를 넣으면 2배 틀린다.
    //
    // 쓰이는 곳은 서보 모드의 ERPM <-> rad/s 환산 두 군데뿐이다.
    //   MIT 모드는 이 값을 쓰지 않는다 — 프로토콜이 +-mit_v_limit 범위를 12비트로
    //   직접 인코딩하므로 극수가 개입하지 않는다.
    const double pole = 21.0;
    double gear_ratio;          // 기어비
    double current_limit;       // 전류 제한
    double control_gain;        // 서보 VEL 모드용 P 게인 (MIT 전환 후에는 미사용, 복귀용으로 유지)

    // ===== MIT 모드 =====
    double mit_kp = 100.0;      // [N·m/rad] 출력축. 학습 액추에이터 stiffness와 같은 값이어야 한다
    double mit_kd = 5.0;        // [N·m·s/rad] CubeMars 매뉴얼 상한 Kd <= 5
    double mit_p_limit = 12.5;  // 인코딩 범위 [rad]
    double mit_v_limit = 50.0;  // 인코딩 범위 [rad/s]
    double mit_t_limit = 25.0;  // 인코딩 범위 [N·m] — 코덱의 12비트 스케일

    // 게인 인코딩 범위. p/v/t 와 달리 우리가 0 이 아닌 실제 값을 보내므로,
    // 이 범위가 모터 펌웨어와 다르면 그 비율만큼 다른 게인이 걸린다.
    //   예: 우리 kd_max=5 로 kd=5 를 보내면 0xFFF 인데, 모터의 KD_MAX 가 100 이면 20배가 걸린다.
    // p_limit 오류는 급변 차단·범위 검사가 잡지만, 게인 오류는 잡을 방법이 없다.
    // 모델·펌웨어마다 다를 수 있으므로 motors.json 에서 모터별로 받는다.
    double mit_kp_max = 500.0;
    double mit_kd_max = 5.0;

    double mit_torque_safety = 24.0;  // 안전 차단 임계 [N·m] — 서보 시절 current_limit 과 같은 역할

    MotorMitLimits mit_limits() const;

    double current_position = 0.0;
    // [rad/s] 출력축. 모드에 무관하게 항상 rad/s 다 —
    // 서보 피드백은 ERPM 으로 오므로 Controller 가 수신 직후 환산해 담는다.
    // (위치가 1:1 이라 모터 rad/s = 관절 rad/s. 부호는 direction_sign 적용 전)
    double current_velocity = 0.0;
    double current_motor_current  = 0.0;   // 서보 모드 피드백 전류 [A] (MIT 모드에서는 미사용)
    double current_torque_mit = 0.0;       // MIT 모드 피드백 토크 [N·m]

    // 과전류 체크 카운터
    int cnt = 0;

    // 수신이 한 번이라도 되었는지 확인
    bool first_recv_done = false;
private:
};

class MaxonMotor : public Motor {
public:
    MaxonMotor(int id);

    // For CAN communication
    uint32_t node_id;
    int socket;
    bool is_connected = false;

    uint32_t can_send_id;
    uint32_t can_receive_id;

    uint32_t tx_pdo_ids[4];
    uint32_t rx_pdo_ids[2];

    double gear_ratio;
    double control_kp;          // CST 모드용 P 게인
    double control_kd;          // CST 모드용 D 게인
    double prev_err = 0.0;       // CST 모드용 이전 오차
    double prev_err_dot = 0.0;   // CST 모드용 이전 오차 미분

    ControlMode mode = ControlMode::CSP;

    double current_position = 0.0;
    double current_torque   = 0.0;
    unsigned char status_bit = 0;
private:
};

class DynamixelMotor : public Motor {
public:
    DynamixelMotor(int id);

    uint8_t dxl_id;

    int32_t angle_to_tick(double angle);
    double tick_to_angle(int32_t ticks);

    double current_position = 0.0;

private:
};