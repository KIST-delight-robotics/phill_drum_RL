#pragma once

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>

#include "common/play_session.hpp"

enum class RobotState { STANDBY, INIT, IDLE, PLAYING, SHUTTINGDOWN };

inline const char* state_to_string(RobotState s) {
    switch (s) {
        case RobotState::STANDBY:      return "STANDBY";
        case RobotState::INIT:         return "INIT";
        case RobotState::IDLE:         return "IDLE";
        case RobotState::PLAYING:      return "PLAYING";
        case RobotState::SHUTTINGDOWN: return "SHUTTINGDOWN";
        default:                       return "UNKNOWN";
    }
}

struct AppContext {
    std::atomic<bool> running{true};                // 전체 종료 플래그 (false 되면 모든 스레드 루프 탈출)
    std::atomic<bool> send_active{false};           // send_loop 활성화 신호
    std::atomic<bool> recv_active{false};           // recv_loop 활성화 신호

    std::atomic<RobotState> robot_state{RobotState::STANDBY};  // 로봇 상태

    // ===== MIT 모드 =====
    std::atomic<bool>   tmotor_mit{true};       // 팔 TMotor를 MIT로 구동할지 (motors.json 최상위에서 로드)
    std::atomic<bool>   mit_enter_requested{false};  // send_loop에게 MIT 제어 모드 진입 프레임 송신을 요청

    // 토크 인가 직전에 궤적 출발점을 실측으로 맞추라는 요청.
    //
    // 왜 필요한가: 궤적은 last_q(초기값 = init 자세)에서 출발한다. 모터가 그 자리에
    // 없으면 첫 목표부터 오차가 있고, MIT 는 오차가 곧 토크이므로 즉시 큰 토크가 걸린다.
    //   실측 2026-09-02: 축이 28도 돌아간 상태에서 START -> Kp 100 x 0.49 rad = 49 N·m
    // 실측에서 출발시키면 오차가 0 에서 시작해 3초 궤적을 따라가는 만큼만 토크가 걸린다.
    // (서보 모드는 오차를 속도로 바꾸므로 이 문제가 드러나지 않았다.)
    std::atomic<bool>   sync_last_q_requested{false};

    // ===== 정책 =====
    std::atomic<bool>     policy_active{false}; // 정책이 팔 0~8을 소유하는 구간인지
    std::atomic<bool>     policy_fault{false};  // 워치독 실패 / 추론 타임아웃

    // 건식 시험. 정책 스레드를 돌려 배선·타이밍만 확인하고 팔에는 아무것도 쓰지 않는다.
    //   - 팔 모터 결번을 허용한다 (벤치에서 모터 1개만 연결한 상태를 위해)
    //   - 악보 없이 next_hits 를 0 으로 채워 obs 를 조립한다
    //   - policy_active 는 서지 않으므로 merge 가 일어나지 않는다 → 모터 무영향
    // motors.json 최상위 "policy_dry_run" 으로 켠다.
    std::atomic<bool>     policy_dry_run{false};

    // PolicyRunner 가 세션·ObsBuilder 초기화를 마치면 true. 이게 false 면
    // policy_active 를 절대 세우지 않는다 — 세우면 팔이 주인 없이 남는다.
    std::atomic<bool>     policy_ready{false};

    // 곡 시작마다 +1. PolicyRunner 가 값이 바뀐 것을 보고 ObsBuilder 를 reset 한다.
    // 재장전 상태머신·스케줄러 히스테리시스가 이전 곡의 상태를 물고 가면 안 된다.
    std::atomic<uint64_t> policy_epoch{0};
    std::atomic<uint64_t> tick{0};              // send_loop이 5ms pop마다 +1. 유일한 시간 권위
    std::atomic<double>   t_score{0.0};         // 현재 tick의 악보 시간 [s]

    std::mutex              policy_mtx;
    std::condition_variable policy_cv;          // send_loop이 POLICY_TICK_STRIDE 틱마다 notify

    std::mutex last_q_mutex;                        // 마지막 목표 관절각 스냅샷 (BehaviorPlanner가 갱신, TcpServer가 조회)
    std::vector<double> last_q_target_snapshot;     // NOTE: 실측값이 아니라 "마지막으로 명령된 목표 자세"임

    std::atomic<bool> play_abort{false};            // play 중 중단 플래그
    std::atomic<bool> pause_requested{false};       // abort 사유 구분 (true: PAUSE=재개 지점 저장 / false: stop·에러=폐기)
    std::atomic<double> play_speed_scale{1.0};      // play 중 속도 스케일

    std::mutex play_mutex;                          // 아래 두 필드 보호 (MotionPlanner가 갱신, 추후 TcpServer가 조회)
    std::string play_id;                            // 연주 중인 곡 id (빈 문자열 = 연주 아님)
    PausePoint pause_point;                         // 중단 시 저장, resume 시 소비
};