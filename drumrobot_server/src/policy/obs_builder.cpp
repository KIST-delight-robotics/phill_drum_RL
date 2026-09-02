#include "policy/obs_builder.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include "nlohmann/json.hpp"

namespace {

// 스케줄러 상수 (drumrobot_env.py::_compute_arm_schedule)
constexpr double BIG = 1.0e3;
// drumrobot_cfg.py — 학습 런의 params/env.yaml 에 있는 값
constexpr double SCHED_CROSS_PENALTY = 1.5;
constexpr double SCHED_HYSTERESIS    = 0.25;

constexpr double NEVER_HIT = -1.0e9;

inline double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

bool ObsBuilder::initialize(const PolicyConfig& cfg, const std::string& drum_coord_path) {
    using json = nlohmann::json;

    ready_ = false;
    cfg_ = cfg;
    if (!cfg_.valid) {
        std::cerr << "[ObsBuilder] PolicyConfig 가 유효하지 않습니다\n";
        return false;
    }

    solver_.initialize();

    std::ifstream f(drum_coord_path);
    if (!f.is_open()) {
        std::cerr << "[ObsBuilder] 열 수 없습니다: " << drum_coord_path << "\n";
        return false;
    }
    json root;
    try {
        f >> root;
    } catch (const json::parse_error& e) {
        std::cerr << "[ObsBuilder] JSON 파싱 실패: " << e.what() << "\n";
        return false;
    }

    // 실기는 악기마다 right/left 좌표를 따로 갖는데 학습은 하나만 쓴다.
    // 전수 대조 결과: 학습 좌표 = midpoint(right, left) - (0, 0, z_shift).
    // XY 오차 0.5mm 이내, Z 는 8개 전부 정확히 +45mm.
    // z 는 정규화 clamp 에 흡수되지만 XY 중점 변환은 흡수되지 않는다.
    std::array<bool, NUM_DRUM> filled{};
    try {
        for (const auto& inst : root.at("instruments")) {
            const std::string name = inst.at("name");
            const auto it = instrument_name_to_id.find(name);
            if (it == instrument_name_to_id.end()) continue;

            int id = it->second;
            if (id == 9) id = 5;                       // open hihat -> closed hihat (결정 02)
            if (id < 1 || id > NUM_DRUM) continue;     // 0(bass) 는 페달, obs 에 없음
            const int d = id - 1;
            if (filled[d]) continue;                   // 먼저 온 것을 유지 (closed 가 open 보다 먼저)

            const auto& R = inst.at("right").at("position");
            const auto& L = inst.at("left").at("position");
            for (int k = 0; k < 3; ++k) {
                drum_[d][k] = 0.5 * (R.at(k).get<double>() + L.at(k).get<double>());
            }
            drum_[d][2] -= cfg_.drum_z_shift;
            filled[d] = true;
        }
    } catch (const json::exception& e) {
        std::cerr << "[ObsBuilder] 드럼 좌표 읽기 실패: " << e.what() << "\n";
        return false;
    }

    for (int d = 0; d < NUM_DRUM; ++d) {
        if (!filled[d]) {
            std::cerr << "[ObsBuilder] 드럼 obs 인덱스 " << d << " (실기 id " << d + 1
                      << ") 좌표가 없습니다\n";
            return false;
        }
    }

    reset();
    ready_ = true;

    std::cout << "[ObsBuilder] 드럼 좌표 " << NUM_DRUM << "개 로드 (중점 - z " << cfg_.drum_z_shift << ")\n";
    for (int d = 0; d < NUM_DRUM; ++d) {
        std::cout << "    obs[" << d << "] id " << d + 1 << "  ("
                  << drum_[d][0] << ", " << drum_[d][1] << ", " << drum_[d][2] << ")\n";
    }
    return true;
}

void ObsBuilder::reset() {
    for (auto& arm : armed_) arm.fill(false);      // 초기 상태는 준비 안됨 (hit_detector.reset)
    for (auto& v : tip_prev_) v.fill(0.0);
    for (auto& v : tip_vel_)  v.fill(0.0);
    tip_seeded_ = false;

    last_hit_step_.fill(NEVER_HIT);
    prev_use_left_h0_ = true;
    prev_arm_role_.fill(0.0f);

    wrist_prev_q_.fill(0.0);
    wrist_prev_ns_ = 0;
    wrist_seeded_ = false;
}

// hit_detector.py 의 _check_contact_drum / _check_hitting / _check_rearm 이식.
void ObsBuilder::update_hit_state(const std::array<std::array<double, 3>, 2>& tip,
                                  double dt, double now_step) {
    // ---- 팁 속도 (저역통과). alpha 는 학습과 같은 값 ----
    if (!tip_seeded_) {
        tip_prev_ = tip;
        tip_seeded_ = true;
    } else if (dt > 1e-9) {
        const double a = cfg_.tip_vel_filter_alpha;
        for (int arm = 0; arm < 2; ++arm) {
            for (int k = 0; k < 3; ++k) {
                const double raw = (tip[arm][k] - tip_prev_[arm][k]) / dt;
                tip_vel_[arm][k] = a * raw + (1.0 - a) * tip_vel_[arm][k];
            }
        }
        tip_prev_ = tip;
    }

    const double r2 = cfg_.drum_xy_radius * cfg_.drum_xy_radius;

    for (int arm = 0; arm < 2; ++arm) {
        // ---- 접촉: xy 반경 안 && 0 <= diff_z <= drum_z_range ----
        // 여러 드럼 반경에 동시에 들어도 "가장 가까운 하나"만 접촉으로 인정한다.
        // 반경 겹침 때문에 한 번의 타격이 인접 드럼까지 성공으로 세는 것을 막는 규칙이다.
        int nearest = -1;
        double best = std::numeric_limits<double>::infinity();
        std::array<double, NUM_DRUM> diff_z{};

        for (int d = 0; d < NUM_DRUM; ++d) {
            const double dx = tip[arm][0] - drum_[d][0];
            const double dy = tip[arm][1] - drum_[d][1];
            diff_z[d] = tip[arm][2] - drum_[d][2];

            const double dxy2 = dx * dx + dy * dy;
            const bool in_xy = dxy2 <= r2;
            const bool in_z  = (diff_z[d] >= 0.0) && (diff_z[d] <= cfg_.drum_z_range);
            if (!(in_xy && in_z)) continue;

            const double d2 = dxy2 + diff_z[d] * diff_z[d];   // 3D 근사거리
            if (d2 < best) { best = d2; nearest = d; }
        }

        // 갱신 전 상태를 남긴다 — 타격 판정은 prev_hit_armed 로 한다 (학습과 같은 순서).
        const std::array<bool, NUM_DRUM> prev = armed_[arm];

        // ---- 재장전: 충분히 벗어나고 올라가면 그 (팔, 드럼) armed ----
        for (int d = 0; d < NUM_DRUM; ++d) {
            if (diff_z[d] > cfg_.rearm_height) armed_[arm][d] = true;
        }

        // ---- 해제: 접촉 중이면 그 팔의 전 드럼 disarm ----
        if (nearest >= 0) armed_[arm].fill(false);

        // ---- 타격 판정: 접촉 && (직전에) armed && 하강 중 ----
        // 학습 _check_hitting 과 같다. 이 값 자체는 obs 에 들어가지 않고,
        // 스케줄러의 "방금 친 팔은 busy" 판정(last_hit_step)에만 쓴다.
        if (nearest >= 0 && prev[nearest] &&
            tip_vel_[arm][2] < -cfg_.min_impact_velocity) {
            last_hit_step_[arm] = now_step;
        }
    }
}

// drumrobot_env.py::_compute_arm_schedule 이식.
//   각 타격을 "현재위치 + 이동가용시간 + 재장전 상태"로 배정한다.
//   score = travel/window + BIG*relu(-window) → argmin.
//   방금 친 팔은 window 가 작아 penalty 가 커지므로 빠른 클러스터에서 양팔이 교대한다.
void ObsBuilder::run_scheduler(const float* nh,
                               const std::array<std::array<double, 3>, 2>& tip,
                               double now_step,
                               PolicyObs& out) {
    const int K = NUM_HITS;
    const int M = NUM_DRUM;
    const double L = static_cast<double>(cfg_.max_lookahead_step);
    const double t_rearm = 2.0 * cfg_.hit_window_step + 1.0;

    // 팔 투영위치 — 시드는 현재 팁
    std::array<std::array<double, 3>, 2> proj = tip;
    // 팔 자유시각(스텝). 방금 친 팔은 재장전 남은 만큼 양수 → busy
    std::array<double, 2> freev{};
    for (int a = 0; a < 2; ++a) {
        freev[a] = std::max(0.0, (last_hit_step_[a] - now_step) + t_rearm);
    }

    std::array<int, 2> count{0, 0};
    out.per_arm_pos.fill(0.0f);
    out.per_arm_time.fill(1.0f);      // 기본 1.0 = 없음/멀리

    bool use_left_h0 = prev_use_left_h0_;
    int  h0_target_count = 0;

    auto arm_score = [&](int a, const std::array<double, 3>& p, double tk) {
        const double w = tk - freev[a];
        // 교차 침범량[m]: 왼팔(a=0)이 우측(x>0) / 오른팔(a=1)이 좌측(x<0) 드럼을 맡을 때만 >0.
        // 드럼 x 와 팔 정체성만으로 결정 — 현재 팁 위치와 무관하게 두어
        // "이미 교차해 travel=0" 자기강화 고리를 끊는다.
        const double cross = (a == 0) ? std::max(0.0, p[0]) : std::max(0.0, -p[0]);
        const double travel = dist3(proj[a], p) + SCHED_CROSS_PENALTY * cross;
        return travel / std::max(w, 1.0) + BIG * std::max(0.0, -w);
    };

    for (int k = 0; k < K; ++k) {
        const float* row = nh + k * CHANNELS;
        if (row[M + 1] <= 0.5f) continue;                 // valid 아님

        // 이벤트당 상위 2개 드럼 — 드럼 인덱스 큰 순 (학습 topk 와 같은 정렬)
        int i0 = -1, i1 = -1;
        for (int d = M - 1; d >= 0; --d) {
            if (row[d] <= 0.5f) continue;
            if (i0 < 0) i0 = d;
            else if (i1 < 0) { i1 = d; break; }
        }
        if (i0 < 0) continue;                             // multi-hot 이 비었음

        const bool two = (i1 >= 0);
        const auto& p0 = drum_[i0];
        const auto& p1 = drum_[two ? i1 : i0];

        const double tn = row[M];
        const double tk = tn * std::max(L - 1.0, 1.0);    // 정규화 -> 스텝

        const double sL0 = arm_score(0, p0, tk);
        const double sR0 = arm_score(1, p0, tk);
        const double sL1 = arm_score(0, p1, tk);
        const double sR1 = arm_score(1, p1, tk);

        bool choose_left;
        if (k == 0) {
            // 히스테리시스는 k=0 에만. 배정에 관성이 없으면 두 팁이 등거리일 때
            // mm 단위로 승자가 뒤집히고, proj 체인 때문에 양팔 큐 전체가 스왑된다.
            const double h = SCHED_HYSTERESIS;
            choose_left = prev_use_left_h0_ ? (sL0 <= sR0 * (1.0 + h))
                                            : (sL0 * (1.0 + h) <= sR0);
        } else {
            choose_left = (sL0 <= sR0);
        }
        const bool useA = (sL0 + sR1) <= (sL1 + sR0);     // 2타: 왼->p0,오른->p1 vs swap

        if (k == 0) {
            h0_target_count = two ? 2 : 1;
            if (!two) use_left_h0 = choose_left;
        }

        const bool left_gets  = two || choose_left;
        const bool right_gets = two || !choose_left;
        const std::array<double, 3>& left_pos  = two ? (useA ? p0 : p1) : p0;
        const std::array<double, 3>& right_pos = two ? (useA ? p1 : p0) : p0;

        for (int a = 0; a < 2; ++a) {
            const bool gets = (a == 0) ? left_gets : right_gets;
            if (!gets || count[a] >= SCHED_NOTES) continue;
            const auto& gp = (a == 0) ? left_pos : right_pos;
            const int s = count[a];
            for (int c = 0; c < 3; ++c) {
                out.per_arm_pos[(a * SCHED_NOTES + s) * 3 + c] = static_cast<float>(gp[c]);
            }
            out.per_arm_time[a * SCHED_NOTES + s] = static_cast<float>(tn);
            ++count[a];
        }

        const double new_free = tk + t_rearm;
        if (left_gets)  { proj[0] = left_pos;  freev[0] = new_free; }
        if (right_gets) { proj[1] = right_pos; freev[1] = new_free; }
    }

    // ---- arm_role ----
    // reward.py::compute_arm_target_assignment 의 순수 부분. 보상값과 무관하다.
    //   h0 드럼 1개 -> 담당 팔만 활성 / 2개 이상 -> 양팔 / 0개 -> 양팔 노는 중
    // 학습은 이 값을 _get_rewards 에서 만들어 다음 obs 에 싣는다 → 1스텝 지연.
    // 골든 테스트를 정확히 맞추기 위해 그 지연을 재현한다.
    out.arm_role = prev_arm_role_;

    std::array<float, 2> role{0.0f, 0.0f};
    if (h0_target_count == 1) {
        role[0] = use_left_h0 ? 1.0f : 0.0f;
        role[1] = use_left_h0 ? 0.0f : 1.0f;
    } else if (h0_target_count >= 2) {
        role[0] = role[1] = 1.0f;
    }
    prev_arm_role_ = role;
    prev_use_left_h0_ = use_left_h0;
}

bool ObsBuilder::build(const JointSnapshot::Data& snap,
                       double t_score,
                       double speed,
                       const PolicyScoreStore& score,
                       PolicyObs& out) {
    if (!ready_) return false;

    std::array<float, NUM_HITS * CHANNELS> nh{};
    if (!score.next_hits(t_score, speed, nh.data())) return false;
    return build_from(snap, t_score, nh.data(), out);
}

bool ObsBuilder::build_from(const JointSnapshot::Data& snap,
                            double t_score,
                            const float* next_hits,
                            PolicyObs& out) {
    if (!ready_) return false;

    // ---- 1. joint_pos / joint_vel : 모터 id 순서 -> obs 순서 ----
    // 이 재배열이 틀리면 정책이 왼손목 명령을 오른어깨로 보낸다.
    std::array<double, JointID::NUM_ARM> q_motor{};
    for (int i = 0; i < JointID::NUM_ARM; ++i) {
        const int m = cfg_.motor_id_by_obs[i];
        out.joint_pos[i] = static_cast<float>(snap.q[m]);
        out.joint_vel[i] = static_cast<float>(snap.qd[m]);
        q_motor[m] = snap.q[m];
    }

    // 손목(모터 7, 8)은 Maxon 이라 속도 피드백이 없다. 스냅샷의 t_pub_ns 로 유한차분한다.
    // 100us 간격 차분은 엔코더 양자화 노이즈가 커서 쓸 수 없다 — 여기서는 정책 주기
    // (15ms) 간격이라 관절 해상도가 1e-3 rad 이어도 노이즈가 0.07 rad/s 수준이다.
    double dt = cfg_.policy_dt;
    if (wrist_seeded_ && snap.t_pub_ns > wrist_prev_ns_) {
        dt = static_cast<double>(snap.t_pub_ns - wrist_prev_ns_) * 1e-9;
    }
    for (int w = 0; w < 2; ++w) {
        const int m = (w == 0) ? JointID::R_WRIST : JointID::L_WRIST;
        double qd = 0.0;
        if (wrist_seeded_ && dt > 1e-6) qd = (snap.q[m] - wrist_prev_q_[w]) / dt;
        wrist_prev_q_[w] = snap.q[m];
        // obs 인덱스를 찾아 넣는다
        for (int i = 0; i < JointID::NUM_ARM; ++i) {
            if (cfg_.motor_id_by_obs[i] == m) { out.joint_vel[i] = static_cast<float>(qd); break; }
        }
    }
    wrist_prev_ns_ = snap.t_pub_ns;
    wrist_seeded_ = true;

    // ---- 2. tip_pos : FK. 학습 obs 는 (왼, 오른) 순서다 ----
    const KinematicsSolver::FKResult fk = solver_.solve_fk(q_motor);
    if (!fk.success) {
        std::cerr << "[ObsBuilder] solve_fk 실패\n";
        return false;
    }
    std::array<std::array<double, 3>, 2> tip{fk.pL, fk.pR};
    if (use_tip_override_) tip = tip_override_;      // 골든 테스트 전용
    for (int a = 0; a < 2; ++a) {
        for (int c = 0; c < 3; ++c) out.tip_pos[a * 3 + c] = static_cast<float>(tip[a][c]);
    }

    // ---- 3. drum_pos : 정적 ----
    for (int d = 0; d < NUM_DRUM; ++d) {
        for (int c = 0; c < 3; ++c) out.drum_pos[d * 3 + c] = static_cast<float>(drum_[d][c]);
    }

    // ---- 4. next_hits (주입값) ----
    std::copy(next_hits, next_hits + NUM_HITS * CHANNELS, out.next_hits.begin());

    // ---- 5. hit_armed : 상태머신 갱신 후 반영 ----
    const double now_step = t_score / cfg_.train_step_dt;
    update_hit_state(tip, dt, now_step);
    for (int a = 0; a < 2; ++a) {
        for (int d = 0; d < NUM_DRUM; ++d) {
            out.hit_armed[a * NUM_DRUM + d] = armed_[a][d] ? 1.0f : 0.0f;
        }
    }

    // ---- 6. 스케줄러 : per_arm_pos / per_arm_time / arm_role ----
    run_scheduler(out.next_hits.data(), tip, now_step, out);

    return true;
}
