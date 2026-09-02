#include "policy/policy_config.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

bool PolicyConfig::load(const std::string& path) {
    using json = nlohmann::json;

    valid = false;
    source_path = path;

    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[PolicyConfig] 열 수 없습니다: " << path << "\n";
        return false;
    }

    json j;
    try {
        f >> j;
    } catch (const json::parse_error& e) {
        std::cerr << "[PolicyConfig] JSON 파싱 실패: " << e.what() << "\n";
        return false;
    }

    try {
        run = j.value("run", std::string("(미기록)"));

        const int obs_dim = j.value("obs_dim", 0);
        const int act_dim = j.value("act_dim", 0);
        if (act_dim != JointID::NUM_ARM) {
            std::cerr << "[PolicyConfig] act_dim 이 " << JointID::NUM_ARM
                      << " 이 아닙니다: " << act_dim << "\n";
            return false;
        }
        if (obs_dim <= 0) {
            std::cerr << "[PolicyConfig] obs_dim 이 없습니다\n";
            return false;
        }

        joint_order = j.at("joint_order").get<std::vector<std::string>>();
        auto ids = j.at("motor_id_by_obs_index").get<std::vector<int>>();
        if (joint_order.size() != JointID::NUM_ARM || ids.size() != JointID::NUM_ARM) {
            std::cerr << "[PolicyConfig] joint_order / motor_id_by_obs_index 길이가 "
                      << JointID::NUM_ARM << " 이 아닙니다\n";
            return false;
        }
        for (int i = 0; i < JointID::NUM_ARM; ++i) {
            if (ids[i] < 0 || ids[i] >= JointID::NUM_ARM) {
                std::cerr << "[PolicyConfig] 모터 id 범위 오류: obs[" << i << "] -> " << ids[i] << "\n";
                return false;
            }
            motor_id_by_obs[i] = ids[i];
        }
        // 순열인지 확인 — 빠지거나 겹치면 관절이 뒤바뀐다
        std::array<bool, JointID::NUM_ARM> seen{};
        for (int id : motor_id_by_obs) {
            if (seen[id]) {
                std::cerr << "[PolicyConfig] 모터 id " << id << " 가 중복됩니다 (순열이 아님)\n";
                return false;
            }
            seen[id] = true;
        }

        policy_dt          = j.value("policy_dt", policy_dt);
        train_step_dt      = j.value("train_step_dt", train_step_dt);
        policy_tick_stride = j.value("policy_tick_stride", policy_tick_stride);
        action_scale       = j.value("action_scale", action_scale);
        wrist_action_scale = j.value("wrist_action_scale", wrist_action_scale);

        if (const auto it = j.find("hit_detector"); it != j.end()) {
            const auto& h = *it;
            drum_xy_radius       = h.value("drum_xy_radius", drum_xy_radius);
            drum_z_range         = h.value("drum_z_range", drum_z_range);
            min_impact_velocity  = h.value("min_impact_velocity", min_impact_velocity);
            rearm_height         = h.value("rearm_height", rearm_height);
            tip_vel_filter_alpha = h.value("tip_vel_filter_alpha", tip_vel_filter_alpha);
            hit_window_step      = h.value("hit_window_step", hit_window_step);
            hit_window_s         = h.value("hit_window_s", hit_window_s);
            rearm_s              = h.value("rearm_s", rearm_s);
        }

        max_lookahead_s = j.value("max_lookahead_s", max_lookahead_s);
        num_hits        = j.value("num_hits", num_hits);
        sched_notes     = j.value("sched_notes", sched_notes);

        // 없으면 학습 식으로 유도하되, 있으면 그 값을 쓴다 (반올림 드리프트 방지)
        max_lookahead_step = j.value("max_lookahead_step",
                                     static_cast<int>(std::lround(max_lookahead_s / train_step_dt)));
        time_norm_denom_s  = j.value("time_norm_denom_s",
                                     (max_lookahead_step - 1) * train_step_dt);
        if (max_lookahead_step < 2 || time_norm_denom_s <= 0.0) {
            std::cerr << "[PolicyConfig] lookahead 값이 이상합니다: step=" << max_lookahead_step
                      << ", denom=" << time_norm_denom_s << "\n";
            return false;
        }

        if (const auto it = j.find("real_to_sim_drum"); it != j.end()) {
            drum_z_shift = it->value("z_shift", drum_z_shift);
        }

        // 그래프가 정규화를 한다는 전제가 깨지면 C++ 이 raw 를 넣는 것이 틀린 동작이 된다
        if (!j.value("graph_does_normalization", false)) {
            std::cerr << "[PolicyConfig] graph_does_normalization 이 false 입니다. "
                      << "이 코드는 그래프가 정규화한다는 전제로 원시 물리량을 넣습니다.\n";
            return false;
        }
        const std::string frame = j.value("position_frame", std::string());
        if (frame != "waist_relative_meters") {
            std::cerr << "[PolicyConfig] position_frame 이 waist_relative_meters 가 아닙니다: "
                      << frame << "\n";
            return false;
        }
    } catch (const json::exception& e) {
        std::cerr << "[PolicyConfig] 필드 읽기 실패: " << e.what() << "\n";
        return false;
    }

    valid = true;
    return true;
}

void PolicyConfig::print() const {
    if (!valid) {
        std::cout << "[PolicyConfig] 로드되지 않았습니다\n";
        return;
    }
    std::cout << "[PolicyConfig] " << source_path << "\n"
              << "    학습 런        : " << run << "\n"
              << "    정책 주기      : " << policy_dt << " s (실기, " << policy_tick_stride << "틱)"
              << " / 학습 " << train_step_dt << " s\n"
              << "    lookahead     : " << max_lookahead_s << " s = " << lookahead_steps()
              << " 스텝, time_norm 분모 " << time_norm_denom() << " s\n"
              << "    action_scale  : " << action_scale << " / 손목 " << wrist_action_scale << " rad/s\n"
              << "    타격 판정      : xy " << drum_xy_radius << " m, z " << drum_z_range
              << " m, rearm " << rearm_height << " m, hit_window " << hit_window_s << " s\n"
              << "    드럼 z 보정    : -" << drum_z_shift << " m (실기 중점 기준)\n"
              << "    관절 매핑      : ";
    for (int i = 0; i < JointID::NUM_ARM; ++i) {
        std::cout << "obs" << i << "->m" << motor_id_by_obs[i] << (i + 1 < JointID::NUM_ARM ? " " : "\n");
    }
}
