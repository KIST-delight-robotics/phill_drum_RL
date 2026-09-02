// obs_builder 확인.  make obs-check SCORE=<곡>
//
// 악보를 로드해 가상 관절 궤적을 흘려보내며 원시 버퍼 9개를 만들고,
// 이어서 policy.onnx 에 넣어 act 9 까지 뽑는다.
// 골든 테스트(4-7) 전에 "돌기는 하는가 / 값이 물리적으로 그럴듯한가"를 본다.
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "policy/obs_builder.hpp"
#include "policy/policy_config.hpp"
#include "policy/policy_score.hpp"

static std::vector<DrumEvent> load_score(const std::string& path) {
    std::vector<DrumEvent> rds;
    std::ifstream f(path);
    if (!f.is_open()) return rds;
    double bpm = 100.0, last_t = 0.0;
    std::string row;
    while (std::getline(f, row)) {
        std::istringstream iss(row);
        std::string item; std::vector<std::string> it;
        while (std::getline(iss, item, '\t')) it.push_back(item);
        if (it.empty() || it[0].empty()) continue;
        if (it[0] == "bpm") { if (it.size() > 1) bpm = std::stod(it[1]); continue; }
        if (it[0] == "end") break;
        if (it.size() < 8) continue;
        try {
            DrumEvent ev;
            ev.bar = std::stoul(it[0]);   ev.beat = std::stod(it[1]);
            ev.note_num_R = std::stoi(it[2]); ev.note_num_L = std::stoi(it[3]);
            ev.velocity_R = std::stoi(it[4]); ev.velocity_L = std::stoi(it[5]);
            ev.is_kick = std::stoi(it[6]) != 0; ev.is_closed_hihat = std::stoi(it[7]) != 0;
            ev.t = ev.beat * 100.0 / bpm + last_t; last_t = ev.t;
            rds.push_back(ev);
        } catch (...) { break; }
    }
    return rds;
}

int main(int argc, char** argv) {
    const std::string cfg_path = (argc > 1) ? argv[1] : "drumrobot_server/data/policy/obs_constants.json";
    const std::string score    = (argc > 2) ? argv[2] : "BasicFillin";
    const std::string model    = (argc > 3) ? argv[3] : "drumrobot_server/data/policy/policy.onnx";

    PolicyConfig cfg;
    if (!cfg.load(cfg_path)) return 1;

    ObsBuilder ob;
    if (!ob.initialize(cfg)) return 1;

    PolicyScoreStore store;
    auto rds = load_score("drumrobot_server/data/scores/" + score + ".txt");
    if (rds.empty()) { fprintf(stderr, "악보 없음: %s\n", score.c_str()); return 1; }
    store.publish(rds, cfg, score);

    // ---- ORT 세션 (정책 스레드와 같은 설정) ----
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "obs_check");
    Ort::SessionOptions opt;
    opt.SetIntraOpNumThreads(1); opt.SetInterOpNumThreads(1);
    opt.SetExecutionMode(ORT_SEQUENTIAL);
    Ort::Session sess(env, model.c_str(), opt);
    auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    PolicyObs o;
    std::array<float, 9> act{};
    const int64_t s_jp[2]={1,9}, s_tip[3]={1,2,3}, s_dr[3]={1,8,3}, s_nh[3]={1,6,11},
                  s_ha[3]={1,2,8}, s_ar[2]={1,2}, s_pp[4]={1,2,2,3}, s_pt[3]={1,2,2}, s_out[2]={1,9};
    std::vector<Ort::Value> ins;
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.joint_pos.data(), 9, s_jp, 2));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.joint_vel.data(), 9, s_jp, 2));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.tip_pos.data(), 6, s_tip, 3));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.drum_pos.data(), 24, s_dr, 3));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.next_hits.data(), 66, s_nh, 3));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.hit_armed.data(), 16, s_ha, 3));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.arm_role.data(), 2, s_ar, 2));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.per_arm_pos.data(), 12, s_pp, 4));
    ins.push_back(Ort::Value::CreateTensor<float>(mem, o.per_arm_time.data(), 4, s_pt, 3));
    auto out_t = Ort::Value::CreateTensor<float>(mem, act.data(), 9, s_out, 2);
    const char* in_n[] = {"joint_pos","joint_vel","tip_pos","drum_pos","next_hits",
                          "hit_armed","arm_role","per_arm_pos","per_arm_time"};
    const char* out_n[] = {"action_mean"};

    // ---- 가상 관절 상태: ready 자세 근처에서 손목만 흔든다 ----
    JointSnapshot::Data snap;
    const double ready_q[9] = {0.0, 1.57, 1.57, 0.6, 0.6, 1.2, 1.2, 0.3, 0.3};
    for (int m = 0; m < 9; ++m) snap.q[m] = ready_q[m];
    snap.valid = true;

    printf("\n%-8s %-28s %-10s %-6s %-22s %s\n",
           "t_score", "next_hits[0] drums", "armed(L,R)", "role", "per_arm_time", "act[0..2]");
    printf("%s\n", std::string(104, '-').c_str());

    const int N = 400;
    std::vector<double> us; us.reserve(N);
    int ok = 0;
    for (int i = 0; i < N; ++i) {
        const double t = 2.40 + i * cfg.policy_dt;
        // 손목을 정책 주기로 흔들어 팁이 위아래로 오가게 한다 (재장전/접촉 유발)
        const double s = std::sin(2.0 * M_PI * i / 20.0);
        snap.q[JointID::R_WRIST] = 0.3 + 0.5 * s;
        snap.q[JointID::L_WRIST] = 0.3 + 0.5 * s;
        snap.t_pub_ns = static_cast<uint64_t>(t * 1e9);

        auto t0 = std::chrono::steady_clock::now();
        const bool built = ob.build(snap, t, 1.0, store, o);
        us.push_back(std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0).count());
        if (!built) continue;
        ++ok;

        sess.Run(Ort::RunOptions{nullptr}, in_n, ins.data(), 9, out_n, &out_t, 1);

        if (i % 40 != 0) continue;
        std::string drums;
        for (int d = 0; d < 8; ++d) drums += (o.next_hits[d] > 0.5f) ? std::to_string(d) + " " : "";
        int aL = 0, aR = 0;
        for (int d = 0; d < 8; ++d) { aL += o.hit_armed[d] > 0.5f; aR += o.hit_armed[8 + d] > 0.5f; }
        printf("%7.3f  %-27s (%d,%d)%5s (%.0f,%.0f)  %.2f %.2f | %.2f %.2f   %+.3f %+.3f %+.3f\n",
               t, drums.empty() ? "(없음)" : drums.c_str(), aL, aR, "",
               o.arm_role[0], o.arm_role[1],
               o.per_arm_time[0], o.per_arm_time[1], o.per_arm_time[2], o.per_arm_time[3],
               act[0], act[1], act[2]);
    }
    std::sort(us.begin(), us.end());
    printf("\nbuild 성공 %d/%d\n", ok, N);
    printf("obs_builder 시간  p50 %.2f us   p99 %.2f us   최대 %.2f us   (예산 15000 us)\n",
           us[N/2], us[N*99/100], us[N-1]);
    return 0;
}
