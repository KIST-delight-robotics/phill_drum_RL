#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>

#include "common/joint_snapshot.hpp"
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "policy/policy_config.hpp"
#include "policy/policy_score.hpp"

// 정책 그래프의 입력 버퍼 9개.
//
// 정규화는 하지 않는다 — policy.onnx 그래프 안에서 한다. 여기서 담는 것은
// 전부 원시 물리량이고, 위치의 단위·프레임은 "허리 기준 미터"다.
//
// 관절 배열의 순서는 obs 순서(USD articulation)다. 실기 모터 id 순서가 아니다.
//   obs  0    1    2    3    4    5    6    7    8
//   모터 0    2    1    5    3    6    4    8    7
// PolicyConfig::motor_id_by_obs 가 그 매핑을 준다.
struct PolicyObs {
    std::array<float, JointID::NUM_ARM> joint_pos{};    // [rad]
    std::array<float, JointID::NUM_ARM> joint_vel{};    // [rad/s]
    std::array<float, 2 * 3>  tip_pos{};        // (2, 3)  L, R  [m]
    std::array<float, 8 * 3>  drum_pos{};       // (8, 3)         [m]
    std::array<float, 6 * 11> next_hits{};      // (K=6, M+3)
    std::array<float, 2 * 8>  hit_armed{};      // (2, 8)  0/1
    std::array<float, 2>      arm_role{};       // 활성 1 / 노는팔 0
    std::array<float, 2 * 2 * 3> per_arm_pos{}; // (2, NOTES=2, 3)  [m]
    std::array<float, 2 * 2>     per_arm_time{};// (2, NOTES=2)     이미 [0,1]
};

// obs 조립기.
//
// 정책 스레드에서만 쓴다. 상태를 들고 있으므로 (재장전 상태머신, 스케줄러 히스테리시스)
// 곡 시작마다 reset() 해야 한다.
//
// 학습 쪽 대응:
//   hit_armed 상태머신      components/hit_detector.py
//   팔 스케줄러             drumrobot_env.py::_compute_arm_schedule
//   arm_role                components/reward.py::compute_arm_target_assignment
//                           (next_hits[0] 의 순수 함수 — 보상값과 무관)
class ObsBuilder {
public:
    static constexpr int NUM_DRUM  = 8;
    static constexpr int NUM_HITS  = 6;    // K
    static constexpr int SCHED_NOTES = 2;  // NOTES
    static constexpr int CHANNELS  = NUM_DRUM + 3;

    // drum_coordinate.json 을 읽어 학습 좌표로 변환해 캐시한다.
    //   학습 좌표 = midpoint(real.right, real.left) - (0, 0, cfg.drum_z_shift)
    bool initialize(const PolicyConfig& cfg,
                    const std::string& drum_coord_path =
                        "drumrobot_server/config/drum_coordinate.json");

    // 곡 시작 시. 재장전 상태와 스케줄러 히스테리시스를 초기화한다.
    void reset();

    // 원시 버퍼를 채운다. 악보가 없으면 false (obs 를 만들 수 없다).
    //   snap  : recv 가 발행한 9관절 일괄 스냅샷 (모터 id 순서)
    //   t_score : ctx.t_score (악보 시간 [s])
    //   speed : ctx.play_speed_scale
    bool build(const JointSnapshot::Data& snap,
               double t_score,
               double speed,
               const PolicyScoreStore& score,
               PolicyObs& out);

    // ===== 테스트 이음새 =====
    // next_hits 를 직접 주입한다. 골든 테스트가 sim 의 값을 그대로 넣어
    // "악보 해석"과 "스케줄러·상태머신"을 분리해 검증하는 데 쓴다.
    // (sim 의 rds 는 에피소드마다 무작위화되어 실기가 재현할 수 없다.)
    bool build_from(const JointSnapshot::Data& snap,
                    double t_score,
                    const float* next_hits,   // K*(M+3) = 66
                    PolicyObs& out);

    // 드럼 좌표를 덮어쓴다. sim 은 per-episode 노이즈(drum_noise_scale=0.02)를
    // 섞으므로, 요소별 비교를 하려면 골든 테스트가 sim 이 쓴 값을 넣어야 한다.
    void set_drum_override(const std::array<std::array<double, 3>, NUM_DRUM>& drum) { drum_ = drum; }

    // 팁 좌표를 덮어쓴다. 골든 테스트가 "FK 규약 차이"와 "상태머신 이식"을
    // 분리해 보는 데 쓴다. 학습 tip_offset(0.385)과 실기 stick(0.373)이 12.9mm 다르고,
    // 그 차이가 diff_z 를 통해 hit_armed 판정 시점을 흔든다.
    void set_tip_override(const std::array<std::array<double, 3>, 2>& tip) {
        tip_override_ = tip; use_tip_override_ = true;
    }

    bool ready() const { return ready_; }

private:
    bool ready_ = false;
    PolicyConfig cfg_{};
    KinematicsSolver solver_;

    // 학습 좌표계의 드럼 위치 (obs 인덱스 0~7 = 실기 id 1~8)
    std::array<std::array<double, 3>, NUM_DRUM> drum_{};

    // ===== hit_armed 상태머신 (hit_detector.py) =====
    //   접촉 : xy 반경 <= drum_xy_radius && 0 <= diff_z <= drum_z_range, 최근접 드럼 하나만
    //   재장전 : diff_z > rearm_height 인 (팔, 드럼) 을 armed = true
    //   해제 : 어느 드럼이든 접촉 중이면 그 팔의 8드럼 전부 armed = false
    //   초기값 : false ("준비 안됨")
    std::array<std::array<bool, NUM_DRUM>, 2> armed_{};

    // 팁 속도 (하강 판정용). alpha 저역통과, hit_detector.py::_compute_tip_velocity
    std::array<std::array<double, 3>, 2> tip_prev_{};
    std::array<std::array<double, 3>, 2> tip_vel_{};
    bool tip_seeded_ = false;

    // ===== 스케줄러 상태 =====
    // 방금 친 팔은 재장전 시간만큼 busy 로 잡혀 클러스터에서 교대가 유도된다.
    std::array<double, 2> last_hit_step_{};   // -1e9 = 아직 안 침
    bool prev_use_left_h0_ = true;            // 히스테리시스 기준
    // arm_role 은 학습이 _get_rewards 에서 만들어 다음 obs 에 싣는다 → 1스텝 지연.
    // 학습 주석은 주기가 빨라 무시 가능하다고 하지만, 골든 테스트를 정확히 맞추려면 재현해야 한다.
    std::array<float, 2> prev_arm_role_{};

    // 손목 qd 유한차분용 (Maxon 은 속도 피드백이 없다)
    std::array<double, 2> wrist_prev_q_{};
    uint64_t wrist_prev_ns_ = 0;
    bool wrist_seeded_ = false;

    std::array<std::array<double, 3>, 2> tip_override_{};
    bool use_tip_override_ = false;

    // 반환: 이번 스텝에 실제 타격이 있었던 팔 (스케줄러 busy 갱신용)
    void update_hit_state(const std::array<std::array<double, 3>, 2>& tip, double dt, double now_step);
    void run_scheduler(const float* next_hits,
                       const std::array<std::array<double, 3>, 2>& tip,
                       double now_step,
                       PolicyObs& out);
};
