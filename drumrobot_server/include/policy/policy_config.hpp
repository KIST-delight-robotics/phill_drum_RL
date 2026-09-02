#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/robot_config.hpp"

// export_policy.py 가 내보낸 obs_constants.json 을 읽는다.
//
// 이 값들을 코드에 박지 않는 이유: 학습 쪽에서 관절 한계·타격 판정 상수·주기를
// 바꾸면 조용히 어긋난다. 재export 만 하면 따라오도록 파일로 받는다.
//
// 정규화 상수는 여기 없다 — policy.onnx 그래프 안에 있다.
// (참고용으로 json 의 _normalization_in_graph 에 남아 있지만 C++ 은 쓰지 않는다.)
struct PolicyConfig {
    bool valid = false;

    std::string source_path;
    std::string run;                    // 어느 학습 런에서 나왔는지

    // ===== 관절 매핑 =====
    // obs/action 의 관절 순서는 USD articulation 순서다.
    // specs.py 의 ctrl_joint_names 목록 순서도, URDF 선언 순서도 아니다.
    std::vector<std::string> joint_order;                       // 9개
    std::array<int, JointID::NUM_ARM> motor_id_by_obs{};        // obs 인덱스 -> 실기 모터 id

    // ===== 주기 =====
    double policy_dt        = 0.015;    // 실기 정책 주기 [s]. 적분에 쓴다 (STRIDE틱 x 5ms)
    double train_step_dt    = 0.015;    // 학습 정책 주기 [s]. 악보 양자화 그리드
    int    policy_tick_stride = 3;

    // ===== 적분 =====
    double action_scale       = 10.0;   // rad/s (팔)
    double wrist_action_scale = 25.0;   // rad/s (손목)

    // ===== 타격 판정 (obs_builder 의 hit_armed 상태머신) =====
    double drum_xy_radius       = 0.13;
    double drum_z_range         = 0.07;
    double min_impact_velocity  = 0.2;
    double rearm_height         = 0.18;
    double tip_vel_filter_alpha = 0.6;
    int    hit_window_step      = 3;
    double hit_window_s         = 0.05;
    double rearm_s              = 0.116667;

    // ===== 악보 =====
    double max_lookahead_s      = 2.0;
    // 스텝 수와 time_norm 분모는 export 가 직접 준다. C++ 에서 나누면 부동소수 반올림으로
    // 119가 나올 수 있고, 분모가 (L-1) 이라 그 오차가 obs 에 그대로 들어간다.
    int    max_lookahead_step   = 120;
    double time_norm_denom_s    = 119.0 / 60.0;
    int    num_hits             = 6;    // K
    int    sched_notes          = 2;

    // ===== 좌표 변환 =====
    // 학습 좌표 = midpoint(실기 right, 실기 left) - (0, 0, drum_z_shift)
    double drum_z_shift = 0.045;

    int    lookahead_steps() const { return max_lookahead_step; }
    double time_norm_denom() const { return time_norm_denom_s; }

    bool load(const std::string& path);
    void print() const;
};
