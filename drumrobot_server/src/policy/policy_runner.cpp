#include "policy/policy_runner.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

namespace {

// 그래프 입력 이름. export_policy.py 가 이 순서·이름으로 내보낸다.
const char* kInNames[9] = {
    "joint_pos", "joint_vel", "tip_pos", "drum_pos", "next_hits",
    "hit_armed", "arm_role", "per_arm_pos", "per_arm_time",
};
const char* kOutName = "action_mean";

// 각 입력의 형상. obs_builder.hpp 의 PolicyObs 와 짝이 맞아야 한다.
const std::vector<std::vector<int64_t>> kInShapes = {
    {1, 9}, {1, 9}, {1, 2, 3}, {1, 8, 3}, {1, 6, 11},
    {1, 2, 8}, {1, 2}, {1, 2, 2, 3}, {1, 2, 2},
};

uint64_t now_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

// ORT 상태를 헤더에서 숨긴다.
struct PolicyRunner::Session {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "policy"};
    Ort::SessionOptions opt;
    std::unique_ptr<Ort::Session> s;
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::RunOptions run_opt{nullptr};

    // 버퍼와 텐서를 한 번만 만든다. 매 주기 할당은 실시간 스레드에서 금물이다.
    std::vector<std::vector<float>> in_bufs;
    std::vector<float> out_buf;
    std::vector<Ort::Value> ins, outs;
    std::vector<const char*> in_cstr{kInNames, kInNames + 9};
    std::vector<const char*> out_cstr{kOutName};
};

PolicyRunner::PolicyRunner(AppContext& ctxRef, Robot& robotRef,
                           const PolicyConfig& cfgRef, PolicyScoreStore& scoreRef,
                           PolicyTarget& targetRef, std::string model_path)
    : ctx(ctxRef), robot(robotRef), cfg(cfgRef), score(scoreRef), target(targetRef),
      model_path_(std::move(model_path)), policy_log("policy") {}

PolicyRunner::~PolicyRunner() = default;

bool PolicyRunner::initialize() {
    if (!cfg.valid) {
        std::cerr << "[PolicyRunner] PolicyConfig 무효 — 정책을 켜지 않습니다\n";
        return false;
    }
    if (!obs_builder.initialize(cfg)) {
        std::cerr << "[PolicyRunner] ObsBuilder 초기화 실패\n";
        return false;
    }

    // ---- 주기 정합 검사 ----
    // 이게 어긋나면 정책이 자기가 학습한 것과 다른 간격으로 적분된다. 조용히 틀리고
    // 로그에도 안 남는 종류의 오류라, 여기서 막는다.
    //
    //   cfg.policy_dt / policy_tick_stride : export 가 만든 값 (obs_constants.json)
    //   ROBOT::POLICY_TICK_STRIDE          : send_loop 이 실제로 깨우는 간격
    if (cfg.policy_tick_stride != ROBOT::POLICY_TICK_STRIDE) {
        std::cerr << "[PolicyRunner] stride 불일치: obs_constants.json "
                  << cfg.policy_tick_stride << " vs 실기 " << ROBOT::POLICY_TICK_STRIDE
                  << ". 재export 하거나 robot_config.hpp 를 맞추세요\n";
        return false;
    }
    if (std::abs(cfg.policy_dt - ROBOT::POLICY_DT_SECOND) > 1e-9) {
        std::cerr << "[PolicyRunner] policy_dt 불일치: obs_constants.json " << cfg.policy_dt
                  << "s vs 실기 " << ROBOT::POLICY_DT_SECOND << "s\n";
        return false;
    }
    // 학습 주기와 실기 주기가 다르면 돌기는 하지만 학습에서 본 적 없는 폐루프가 된다.
    // 막지는 않고 크게 남긴다 — 재학습 전 과도기에는 이 상태로 검증할 수 있다.
    const double rel = std::abs(cfg.policy_dt - cfg.train_step_dt) / cfg.train_step_dt;
    if (rel > 1e-6) {
        std::cerr << "[PolicyRunner] 경고: 학습 주기 " << cfg.train_step_dt * 1000.0
                  << "ms 와 실기 주기 " << cfg.policy_dt * 1000.0 << "ms 가 "
                  << rel * 100.0 << "% 다릅니다\n";
    }

    // ---- obs 인덱스별 action_scale ----
    // 학습: wrist_action_scale if "wrist" in name else action_scale
    for (int i = 0; i < JointID::NUM_ARM; ++i) {
        const std::string& name = cfg.joint_order[i];
        action_scale_[i] = (name.find("wrist") != std::string::npos)
                               ? cfg.wrist_action_scale : cfg.action_scale;
    }

    // ---- 관절 한계 (모터 id 순서) ----
    for (int j = 0; j < JointID::NUM_ARM; ++j) {
        auto it = robot.motors.find(j);
        if (it == robot.motors.end()) {
            std::cerr << "[PolicyRunner] 팔 모터 " << j << " 결번 — 정책을 켜지 않습니다\n";
            return false;
        }
        q_min_[j] = it->second->min_angle;
        q_max_[j] = it->second->max_angle;
        if (!(q_min_[j] < q_max_[j])) {
            std::cerr << "[PolicyRunner] 모터 " << j << " 관절 한계가 뒤집혔거나 비었습니다 ("
                      << q_min_[j] << ", " << q_max_[j] << ")\n";
            return false;
        }
    }

    // ---- ORT 세션 ----
    auto sess = std::make_unique<Session>();
    sess->opt.SetIntraOpNumThreads(1);
    sess->opt.SetInterOpNumThreads(1);
    sess->opt.SetExecutionMode(ORT_SEQUENTIAL);

    try {
        sess->s = std::make_unique<Ort::Session>(sess->env, model_path_.c_str(), sess->opt);
    } catch (const Ort::Exception& e) {
        std::cerr << "[PolicyRunner] 세션 생성 실패: " << e.what() << "\n";
        return false;
    }

    // 그래프가 기대한 입출력인지 확인한다. 여기서 안 잡으면 Run 이 던진다.
    if (sess->s->GetInputCount() != 9 || sess->s->GetOutputCount() != 1) {
        std::cerr << "[PolicyRunner] 입출력 개수가 다릅니다: 입력 "
                  << sess->s->GetInputCount() << ", 출력 " << sess->s->GetOutputCount()
                  << " (기대 9, 1)\n";
        return false;
    }
    {
        Ort::AllocatorWithDefaultOptions alloc;
        for (size_t i = 0; i < 9; ++i) {
            auto nm = sess->s->GetInputNameAllocated(i, alloc);
            if (std::string(nm.get()) != kInNames[i]) {
                std::cerr << "[PolicyRunner] 입력 " << i << " 이름이 다릅니다: "
                          << nm.get() << " (기대 " << kInNames[i] << ")\n";
                return false;
            }
        }
        auto on = sess->s->GetOutputNameAllocated(0, alloc);
        if (std::string(on.get()) != kOutName) {
            std::cerr << "[PolicyRunner] 출력 이름이 다릅니다: " << on.get()
                      << " (기대 " << kOutName << ")\n";
            return false;
        }
    }

    for (size_t i = 0; i < 9; ++i) {
        const auto& sh = kInShapes[i];
        int64_t n = std::accumulate(sh.begin(), sh.end(), int64_t{1}, std::multiplies<int64_t>());
        sess->in_bufs.emplace_back(static_cast<size_t>(n), 0.0f);
    }
    sess->out_buf.assign(JointID::NUM_ARM, 0.0f);

    for (size_t i = 0; i < 9; ++i) {
        sess->ins.push_back(Ort::Value::CreateTensor<float>(
            sess->mem, sess->in_bufs[i].data(), sess->in_bufs[i].size(),
            kInShapes[i].data(), kInShapes[i].size()));
    }
    const std::vector<int64_t> out_shape{1, JointID::NUM_ARM};
    sess->outs.push_back(Ort::Value::CreateTensor<float>(
        sess->mem, sess->out_buf.data(), sess->out_buf.size(),
        out_shape.data(), out_shape.size()));

    // 워밍업. 첫 Run 은 커널 준비 때문에 수 ms 걸릴 수 있다 — 임계 경로 밖에서 치른다.
    try {
        for (int i = 0; i < 50; ++i) {
            sess->s->Run(sess->run_opt, sess->in_cstr.data(), sess->ins.data(), 9,
                         sess->out_cstr.data(), sess->outs.data(), 1);
        }
    } catch (const Ort::Exception& e) {
        std::cerr << "[PolicyRunner] 워밍업 실패: " << e.what() << "\n";
        return false;
    }

    sess_ = std::move(sess);
    ctx.policy_ready = true;

    std::cerr << "[PolicyRunner] 준비 완료 — 주기 " << cfg.policy_dt * 1000.0 << "ms ("
              << 1.0 / cfg.policy_dt << "Hz), stride " << cfg.policy_tick_stride
              << ", 모델 " << model_path_ << "\n";
    return true;
}

void PolicyRunner::run() {
    if (!ctx.policy_ready.load()) {
        std::cerr << "[PolicyRunner] 미준비 상태 — 스레드가 아무것도 하지 않습니다\n";
        return;
    }

    std::vector<double> infer_us;   // 종료 시 요약용
    infer_us.reserve(1 << 16);

    while (ctx.running.load()) {
        uint64_t t;
        {
            std::unique_lock<std::mutex> lk(ctx.policy_mtx);
            // 타임아웃을 둔다. 정책 구간이 아니면 notify 가 오지 않으므로
            // 이게 없으면 종료 시 스레드가 깨지 않는다.
            ctx.policy_cv.wait_for(lk, std::chrono::milliseconds(50));
            t = ctx.tick.load();
        }

        if (!ctx.running.load()) break;
        if (!ctx.policy_active.load()) continue;

        // 곡이 바뀌었으면 상태머신을 초기화한다.
        const uint64_t ep = ctx.policy_epoch.load();
        if (ep != seen_epoch_) {
            obs_builder.reset();
            seen_epoch_ = ep;
            std::cerr << "[PolicyRunner] epoch " << ep << " — ObsBuilder reset\n";
        }

        const uint64_t t0 = now_ns();
        const bool ok = step(t);
        if (ok) infer_us.push_back((now_ns() - t0) * 1e-3);
    }

    if (!infer_us.empty()) {
        std::sort(infer_us.begin(), infer_us.end());
        const double mean =
            std::accumulate(infer_us.begin(), infer_us.end(), 0.0) / infer_us.size();
        std::cerr << "[PolicyRunner] " << infer_us.size() << "주기  평균 " << mean
                  << "us  p50 " << infer_us[infer_us.size() / 2]
                  << "us  p99 " << infer_us[infer_us.size() * 99 / 100]
                  << "us  최대 " << infer_us.back() << "us  (예산 "
                  << cfg.policy_dt * 1e6 << "us)\n";
    }
    std::cerr << "[PolicyRunner] 스레드 종료\n";
}

bool PolicyRunner::step(uint64_t t) {
    const JointSnapshot::Data snap = robot.joint_snapshot.read();
    if (!snap.valid) return false;      // 아직 피드백이 없다. 워치독이 잡는다.

    const double t_score = ctx.t_score.load();
    const double speed   = ctx.play_speed_scale.load();

    // ---- obs ----
    const uint64_t tb0 = now_ns();
    PolicyObs obs;
    if (!obs_builder.build(snap, t_score, speed, score, obs)) {
        // 악보가 없거나 FK 실패. 슬롯을 갱신하지 않는다.
        static int warn = 0;
        if (warn++ % 200 == 0) std::cerr << "[PolicyRunner] obs 생성 실패\n";
        return false;
    }
    const double build_us = (now_ns() - tb0) * 1e-3;

    // ---- 입력 버퍼에 복사 ----
    Session& S = *sess_;
    std::copy(obs.joint_pos.begin(),    obs.joint_pos.end(),    S.in_bufs[0].begin());
    std::copy(obs.joint_vel.begin(),    obs.joint_vel.end(),    S.in_bufs[1].begin());
    std::copy(obs.tip_pos.begin(),      obs.tip_pos.end(),      S.in_bufs[2].begin());
    std::copy(obs.drum_pos.begin(),     obs.drum_pos.end(),     S.in_bufs[3].begin());
    std::copy(obs.next_hits.begin(),    obs.next_hits.end(),    S.in_bufs[4].begin());
    std::copy(obs.hit_armed.begin(),    obs.hit_armed.end(),    S.in_bufs[5].begin());
    std::copy(obs.arm_role.begin(),     obs.arm_role.end(),     S.in_bufs[6].begin());
    std::copy(obs.per_arm_pos.begin(),  obs.per_arm_pos.end(),  S.in_bufs[7].begin());
    std::copy(obs.per_arm_time.begin(), obs.per_arm_time.end(), S.in_bufs[8].begin());

    // ---- 추론 ----
    const uint64_t ti0 = now_ns();
    try {
        S.s->Run(S.run_opt, S.in_cstr.data(), S.ins.data(), 9,
                 S.out_cstr.data(), S.outs.data(), 1);
    } catch (const Ort::Exception& e) {
        std::cerr << "[PolicyRunner] 추론 실패: " << e.what() << "\n";
        return false;
    }
    const double infer_us = (now_ns() - ti0) * 1e-3;

    // ---- 적분: q_target = q_now + a * scale * dt ----
    // 스케일과 dt 는 obs 순서 인덱스로, 관절각은 모터 id 로 다룬다.
    // 학습(drumrobot_env.py:374)과 같은 식이고, dt 는 학습의 step_dt 가 아니라
    // 실기 주기(cfg.policy_dt)를 쓴다 — 그래야 지령 각속도가 보존된다.
    std::array<double, JointID::NUM_ARM> q_cmd{};
    for (int i = 0; i < JointID::NUM_ARM; ++i) {
        const float a = S.out_buf[i];
        if (!std::isfinite(a)) {
            std::cerr << "[PolicyRunner] 액션에 NaN/Inf (obs 인덱스 " << i << ")\n";
            return false;
        }
        const int m = cfg.motor_id_by_obs[i];     // obs 순서 -> 모터 id
        double q = snap.q[m] + static_cast<double>(a) * action_scale_[i] * cfg.policy_dt;
        q_cmd[m] = std::clamp(q, q_min_[m], q_max_[m]);
    }

    target.publish(q_cmd, t);
    record(t, t_score, q_cmd, build_us, infer_us);
    return true;
}

void PolicyRunner::record(uint64_t t, double t_score,
                          const std::array<double, JointID::NUM_ARM>& q,
                          double build_us, double infer_us) {
    std::vector<double> row;
    row.reserve(4 + JointID::NUM_ARM);
    row.push_back(static_cast<double>(t));
    row.push_back(t_score);
    row.push_back(build_us);
    row.push_back(infer_us);
    for (int j = 0; j < JointID::NUM_ARM; ++j) row.push_back(q[j]);
    policy_log.record(row);
}
