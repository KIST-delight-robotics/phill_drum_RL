// 골든 테스트 — sim 덤프와 C++ obs_builder 를 요소별로 대조한다.  make golden-check
//
// 이게 단계 4를 "승부처"라 부른 이유다. 스케줄러 90여 줄과 재장전 상태머신을
// 손으로 옮겼으니, 실기로 가기 전에 여기서 틀린 곳을 잡아야 한다.
// 실기에서 처음 발견하면 "obs 조립 버그 / 규약 불일치 / sim2real 간극"을 구분할 수 없다.
//
// 분리 전략
//   - next_hits 는 sim 값을 그대로 주입한다 (sim 의 rds 는 에피소드마다 무작위화되어
//     실기가 재현할 수 없다). 악보 해석은 make score-check 가 따로 본다.
//   - drum_pos 도 sim 값으로 덮어쓴다 (per-episode 노이즈).
//   -> 남는 검증 대상: joint 재배열, FK(tip_pos), hit_armed 상태머신, 스케줄러, arm_role
//
// 프레임: 덤프의 tip/drum/per_arm 은 env 프레임(+z_offset). C++ 은 허리 기준이라
//         비교 전에 z 에서 z_offset 을 뺀다.
#include <algorithm>
#include <utility>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "policy/obs_builder.hpp"
#include "policy/policy_config.hpp"

using json = nlohmann::json;

namespace {

struct Stat {
    std::string name;
    int n = 0;
    int bad = 0;              // 허용오차 초과 요소 수 — 0/1 버퍼는 이게 있어야 진단이 된다
    int bad_steps = 0;        // 하나라도 틀린 스텝 수
    int first_bad_step = -1;
    double max_abs = 0.0;
    int worst_step = -1;
    int worst_idx = -1;
    double worst_got = 0.0, worst_exp = 0.0;
};

double g_tol = 1e-4;

// z 성분마다 offset 을 빼야 하는 버퍼는 stride 3 의 3번째 채널이다.
void compare(Stat& st, int step, const float* got, const std::vector<float>& exp,
             int stride_z = 0, double z_off = 0.0,
             const std::vector<bool>* mask = nullptr) {
    const int n = static_cast<int>(exp.size());
    bool step_bad = false;
    for (int i = 0; i < n; ++i) {
        if (mask && !(*mask)[i]) continue;      // 유효하지 않은 요소는 건너뜀
        double e = exp[i];
        if (stride_z > 0 && (i % stride_z) == 2) e -= z_off;
        const double d = std::abs(static_cast<double>(got[i]) - e);
        if (d > g_tol) {
            ++st.bad; step_bad = true;
            if (st.first_bad_step < 0) st.first_bad_step = step;
        }
        if (d > st.max_abs) {
            st.max_abs = d; st.worst_step = step; st.worst_idx = i;
            st.worst_got = got[i]; st.worst_exp = e;
        }
        ++st.n;
    }
    if (step_bad) ++st.bad_steps;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string cfg_path = (argc > 1) ? argv[1] : "drumrobot_server/data/policy/obs_constants.json";
    const std::string gold_path = (argc > 2) ? argv[2] : "drumrobot_server/data/policy/golden.json";
    const double tol = (argc > 3) ? std::stod(argv[3]) : 1e-4;
    // 4번째 인자가 "simtip" 이면 sim 의 tip_pos 를 주입해 FK 규약 차이를 배제한다.
    const bool use_sim_tip = (argc > 4) && (std::string(argv[4]) == "simtip");
    g_tol = tol;

    PolicyConfig cfg;
    if (!cfg.load(cfg_path)) return 1;

    std::ifstream f(gold_path);
    if (!f.is_open()) { fprintf(stderr, "골든 파일 없음: %s\n", gold_path.c_str()); return 1; }
    json g;
    try { f >> g; } catch (const json::parse_error& e) {
        fprintf(stderr, "JSON 파싱 실패: %s\n", e.what()); return 1;
    }

    const double z_off   = g.value("z_offset", 1.0);
    const double step_dt = g.value("step_dt", 1.0 / 60.0);
    const auto motor_by_obs = g.at("motor_id_by_obs_index").get<std::vector<int>>();

    printf("골든: %s\n", gold_path.c_str());
    printf("  스텝 %d, step_dt %.6f, z_offset %.3f\n",
           g.value("num_steps", 0), step_dt, z_off);

    // 관절 순서가 export 와 같은지 먼저 확인 — 여기서 다르면 뒤가 전부 무의미하다
    bool order_ok = (static_cast<int>(motor_by_obs.size()) == JointID::NUM_ARM);
    for (int i = 0; order_ok && i < JointID::NUM_ARM; ++i)
        order_ok = (motor_by_obs[i] == cfg.motor_id_by_obs[i]);
    printf("  관절 순서 일치: %s\n", order_ok ? "OK" : "불일치 !!");
    printf("  팁 좌표: %s\n", use_sim_tip ? "sim 값 주입 (FK 규약 차이 배제)" : "C++ solve_fk");
    if (!order_ok) {
        printf("    골든 : ");  for (int v : motor_by_obs) printf("%d ", v);
        printf("\n    설정 : ");
        for (int i = 0; i < JointID::NUM_ARM; ++i) printf("%d ", cfg.motor_id_by_obs[i]);
        printf("\n");
        return 2;
    }

    ObsBuilder ob;
    if (!ob.initialize(cfg)) return 1;
    ob.reset();

    Stat s_jp{"joint_pos"}, s_jv{"joint_vel"}, s_tip{"tip_pos"}, s_dr{"drum_pos"},
         s_ha{"hit_armed"}, s_role{"arm_role"}, s_pp{"per_arm_pos"}, s_pt{"per_arm_time"};

    JointSnapshot::Data snap;
    snap.valid = true;
    PolicyObs o;
    int done = 0;

    for (const auto& st : g.at("steps")) {
        const int step = st.value("step", 0);
        const double t = st.value("t_score", 0.0);
        const auto jp = st.at("joint_pos_motor_order").get<std::vector<float>>();
        const auto jv = st.at("joint_vel_motor_order").get<std::vector<float>>();
        const auto& raw = st.at("raw");

        // sim 의 드럼 좌표로 덮어쓴다 (노이즈 포함, env -> 허리 기준)
        const auto dr = raw.at("drum_pos").get<std::vector<float>>();
        std::array<std::array<double, 3>, ObsBuilder::NUM_DRUM> drum{};
        for (int d = 0; d < ObsBuilder::NUM_DRUM; ++d) {
            drum[d][0] = dr[d * 3 + 0];
            drum[d][1] = dr[d * 3 + 1];
            drum[d][2] = dr[d * 3 + 2] - z_off;
        }
        ob.set_drum_override(drum);

        for (int m = 0; m < JointID::NUM_ARM; ++m) { snap.q[m] = jp[m]; snap.qd[m] = jv[m]; }
        snap.t_pub_ns = static_cast<uint64_t>(t * 1e9);

        if (use_sim_tip) {
            const auto tp = raw.at("tip_pos").get<std::vector<float>>();
            std::array<std::array<double, 3>, 2> tip{};
            for (int a = 0; a < 2; ++a) {
                tip[a][0] = tp[a * 3 + 0];
                tip[a][1] = tp[a * 3 + 1];
                tip[a][2] = tp[a * 3 + 2] - z_off;
            }
            ob.set_tip_override(tip);
        }

        const auto nh = raw.at("next_hits").get<std::vector<float>>();
        if (!ob.build_from(snap, t, nh.data(), o)) continue;
        ++done;

        compare(s_jp,   step, o.joint_pos.data(),    raw.at("joint_pos").get<std::vector<float>>());
        compare(s_tip,  step, o.tip_pos.data(),      raw.at("tip_pos").get<std::vector<float>>(), 3, z_off);
        compare(s_dr,   step, o.drum_pos.data(),     raw.at("drum_pos").get<std::vector<float>>(), 3, z_off);
        compare(s_ha,   step, o.hit_armed.data(),    raw.at("hit_armed").get<std::vector<float>>());
        compare(s_role, step, o.arm_role.data(),     raw.at("arm_role").get<std::vector<float>>());
        {
            const auto pt = raw.at("per_arm_time").get<std::vector<float>>();
            std::vector<bool> mask(12, false);
            for (int a = 0; a < 2; ++a)
                for (int sl = 0; sl < 2; ++sl) {
                    const bool has = pt[a * 2 + sl] < 0.999f;   // 1.0 = 타겟 없음
                    for (int c = 0; c < 3; ++c) mask[(a * 2 + sl) * 3 + c] = has;
                }
            compare(s_pp, step, o.per_arm_pos.data(),
                    raw.at("per_arm_pos").get<std::vector<float>>(), 3, z_off, &mask);
        }
        compare(s_pt,   step, o.per_arm_time.data(), raw.at("per_arm_time").get<std::vector<float>>());
        // joint_vel: 손목은 C++ 이 유한차분하므로 팔 7축만 비교한다
        {
            const auto e = raw.at("joint_vel").get<std::vector<float>>();
            for (int i = 0; i < JointID::NUM_ARM; ++i) {
                const int m = cfg.motor_id_by_obs[i];
                if (m == JointID::R_WRIST || m == JointID::L_WRIST) continue;
                const double d = std::abs(o.joint_vel[i] - e[i]);
                if (d > s_jv.max_abs) {
                    s_jv.max_abs = d; s_jv.worst_step = step; s_jv.worst_idx = i;
                    s_jv.worst_got = o.joint_vel[i]; s_jv.worst_exp = e[i];
                }
                ++s_jv.n;
            }
        }
    }

    // ===== 타격 성공률 =====
    // "obs 는 맞는데 정책이 못 친다" 를 즉시 구분하기 위한 집계.
    // sim 의 detect_hit 은 타격을 인정하면 그 팔의 8드럼을 전부 disarm 한다.
    // 따라서 팔 단위 armed 합계의 (>0 -> 0) 전이가 타격의 서명이다.
    //
    // 주의: 이건 "쳤다" 이지 "맞는 드럼을 쳤다" 가 아니다. 아래에서 악보가
    // 요구한 시각·드럼과 대조해 정타/오타를 나눈다.
    {
        const auto& steps = g.at("steps");
        int prev_armed[2] = {-1, -1};
        std::vector<std::pair<int,int>> hits;         // (step, arm)
        std::vector<std::pair<int,std::vector<int>>> want;  // (step, drums)

        for (size_t si = 0; si < steps.size(); ++si) {
            const auto& raw = steps[si].at("raw");
            const auto ha = raw.at("hit_armed").get<std::vector<float>>();
            for (int a = 0; a < 2; ++a) {
                int sum = 0;
                for (int d = 0; d < 8; ++d) sum += (ha[a * 8 + d] > 0.5f) ? 1 : 0;
                if (prev_armed[a] > 0 && sum == 0) hits.push_back({(int)si, a});
                prev_armed[a] = sum;
            }
            // 악보가 이번 스텝에 요구한 타격 (next_hits[k] 의 time 이 0 스텝)
            const auto nh = raw.at("next_hits").get<std::vector<float>>();
            for (int k = 0; k < 6; ++k) {
                const int b = k * 11;
                if (nh[b + 9] <= 0.5f) continue;
                if (nh[b + 8] * (cfg.max_lookahead_step - 1) >= 0.5f) continue;
                std::vector<int> ds;
                for (int d = 0; d < 8; ++d) if (nh[b + d] > 0.5f) ds.push_back(d);
                if (!want.empty() && (int)si - want.back().first <= 1) break;
                want.push_back({(int)si, ds});
                break;
            }
        }

        // 요구 타격마다 ±hit_window_step 안에 타격이 있었나
        const int W = cfg.hit_window_step;
        int matched = 0;
        std::vector<bool> used(hits.size(), false);
        for (const auto& [ws, ds] : want) {
            for (size_t h = 0; h < hits.size(); ++h) {
                if (used[h]) continue;
                if (std::abs(hits[h].first - ws) <= W) { used[h] = true; ++matched; break; }
            }
        }
        const int extra = (int)hits.size() - matched;

        printf("\n타격 집계 (sim 판정 재현)\n");
        printf("  악보 요구        %d회\n", (int)want.size());
        printf("  타격 감지        %d회   (왼 %d / 오른 %d)\n", (int)hits.size(),
               (int)std::count_if(hits.begin(), hits.end(), [](auto& x){ return x.second == 0; }),
               (int)std::count_if(hits.begin(), hits.end(), [](auto& x){ return x.second == 1; }));
        printf("  창(±%d스텝) 안 매칭  %d회  -> %.0f%%\n", W, matched,
               want.empty() ? 0.0 : 100.0 * matched / want.size());
        if (extra > 0) printf("  창 밖 타격       %d회  (오타 또는 리바운드)\n", extra);
        if (hits.empty() && !want.empty())
            printf("  ** 타격 0회 — obs 가 맞아도 정책이 못 치고 있습니다.\n"
                   "     정규화 상수(그래프 내부)를 의심하세요. verify_export.py 참조\n");
    }

    printf("\n대조 결과 (%d 스텝, 허용오차 %.1e)\n", done, tol);
    printf("  %-14s %8s %10s %10s %12s  %s\n",
           "버퍼", "요소수", "틀린수", "틀린스텝", "최대오차", "최악 지점");
    printf("  %s\n", std::string(96, '-').c_str());
    int fail = 0;
    for (const Stat* st : {&s_jp, &s_jv, &s_tip, &s_dr, &s_ha, &s_role, &s_pp, &s_pt}) {
        const bool ok = (st->bad == 0);
        if (!ok) ++fail;
        printf("  %-14s %8d %10d %10d %12.3e  %s",
               st->name.c_str(), st->n, st->bad, st->bad_steps, st->max_abs,
               ok ? "OK" : "불일치");
        if (!ok) printf("  첫 step %d / 최악 step %d idx %d : got %+.6f exp %+.6f",
                        st->first_bad_step, st->worst_step, st->worst_idx,
                        st->worst_got, st->worst_exp);
        printf("\n");
    }
    printf("\n%s — 불일치 버퍼 %d개\n", fail == 0 ? "통과" : "실패", fail);
    return fail == 0 ? 0 : 3;
}
