#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/app_context.hpp"
#include "common/robot_config.hpp"
#include "hardware/robot.hpp"
#include "policy/obs_builder.hpp"
#include "policy/policy_config.hpp"
#include "policy/policy_score.hpp"
#include "policy/policy_target.hpp"
#include "util/logger.hpp"

// 정책 스레드.
//
// 주기의 출처: 자기 타이머가 없다. send_loop 이 POLICY_TICK_STRIDE 틱마다
// ctx.policy_cv 를 notify 하고, 이 스레드는 거기서 깬다. 그래서 정책 주기는
// "몇 밀리초" 가 아니라 "send 틱 몇 개" 로 정의되고, send 가 지터를 겪어도
// 정렬이 유지된다 — 두 개의 독립된 시계가 생기지 않는다.
//
// 한 주기에 하는 일
//   1. JointSnapshot 을 읽는다 (recv 가 9관절을 한 번에 발행한 시간 정합 스냅샷)
//   2. ObsBuilder 로 원시 버퍼 9개를 만든다 (정규화는 그래프 안에서 한다)
//   3. ONNX 추론 -> 관절 속도 지령 9개 (obs 순서)
//   4. q_target = q_now + a * scale * policy_dt, 관절 한계로 clip
//   5. PolicyTarget 슬롯에 최신값으로 덮어쓴다 (큐가 아니다)
//
// 실패하면 슬롯을 갱신하지 않는다. send_loop 의 워치독이 낡음을 보고 잡는다 —
// 여기서 팔에 뭔가를 쓰는 것보다 안전하다.
class PolicyRunner {
public:
    PolicyRunner(AppContext& ctxRef, Robot& robotRef,
                 const PolicyConfig& cfgRef, PolicyScoreStore& scoreRef,
                 PolicyTarget& targetRef,
                 std::string model_path = "drumrobot_server/data/policy/policy.onnx");
    ~PolicyRunner();

    // 세션 로드 + ObsBuilder 초기화 + 워밍업. 성공하면 ctx.policy_ready 를 세운다.
    // 실시간 스레드 진입 전에 부른다 — 지연 초기화를 임계 경로에서 빼기 위해서다.
    bool initialize();

    void run();

private:
    AppContext& ctx;
    Robot& robot;
    const PolicyConfig& cfg;
    PolicyScoreStore& score;
    PolicyTarget& target;
    std::string model_path_;

    ObsBuilder obs_builder;

    // ORT 는 헤더를 여기 끌어들이지 않는다 (컴파일 시간 + main 에서 안 보이게).
    struct Session;
    std::unique_ptr<Session> sess_;

    // obs 인덱스별 action_scale. 학습은 관절 이름에 "wrist" 가 있으면 큰 값을 쓴다.
    std::array<double, JointID::NUM_ARM> action_scale_{};

    // 관절 한계 (모터 id 순서). motors.json 의 min_angle/max_angle.
    std::array<double, JointID::NUM_ARM> q_min_{};
    std::array<double, JointID::NUM_ARM> q_max_{};

    uint64_t seen_epoch_ = 0;

    bool step(uint64_t t);

    // ===== log =====
    Logger policy_log;
    void record(uint64_t t, double t_score, const std::array<double, JointID::NUM_ARM>& q,
                double build_us, double infer_us);
};
