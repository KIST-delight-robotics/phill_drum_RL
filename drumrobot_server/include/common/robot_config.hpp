#pragma once

#include <array>
#include <string>
#include <unordered_map>

namespace ROBOT {
    inline constexpr int    NUM_JOINT      = 13;
    inline constexpr int    NUM_INSTRUMENT = 10;
    inline constexpr double DT_SECOND      = 0.005;

    // 정책 주기 = POLICY_TICK_STRIDE x DT_SECOND. 학습 주기와 같아야 한다.
    //   3 -> 15ms  = 66.7Hz  <- 현재. 학습 SIM_DT 0.005 x decimation 3
    //   2 -> 10ms  = 100Hz   <- 시도했으나 성공률이 떨어져 되돌림 (2026-09-01)
    // 바꾸면 export_policy.py 의 policy_tick_stride 도 같이 바꾸고 재export 해야 한다.
    // PolicyRunner::initialize() 가 obs_constants.json 과 대조해 어긋나면 거절한다.
    inline constexpr int    POLICY_TICK_STRIDE = 3;
    inline constexpr double POLICY_DT_SECOND   = POLICY_TICK_STRIDE * DT_SECOND;
}

// 관절 ID 상수 (motors.json 참조)
namespace JointID {
    constexpr int WAIST            = 0;
    constexpr int R_SHOULDER_1     = 1;
    constexpr int L_SHOULDER_1     = 2;
    constexpr int R_SHOULDER_2     = 3;
    constexpr int R_ELBOW          = 4;
    constexpr int L_SHOULDER_2     = 5;
    constexpr int L_ELBOW          = 6;
    constexpr int R_WRIST          = 7;
    constexpr int L_WRIST          = 8;
    constexpr int R_PEDAL          = 9;
    constexpr int L_PEDAL          = 10;
    constexpr int HEAD_YAW         = 11;
    constexpr int HEAD_PITCH       = 12;

    // 정책이 담당하는 팔 관절 개수 (0 ~ 8)
    constexpr int NUM_ARM          = 9;
}

struct InstrumentCoordinate {
    // 드럼 위치
    // 드럼을 치는 순간 손목 각도
    std::array<double, 3> right_position;
    double                right_wrist_angle;
    std::array<double, 3> left_position;
    double                left_wrist_angle;
};

static const std::unordered_map<std::string, int> instrument_name_to_id = {
    {"bass",         0},
    {"snare",        1},
    {"floor",        2},
    {"mid",          3},
    {"top",          4},
    {"closed hihat", 5},
    {"ride",         6},
    {"right crash",  7},
    {"left crash",   8},
    {"open hihat",   9},
};