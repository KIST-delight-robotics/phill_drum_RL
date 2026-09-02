#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "common/motion_queue.hpp"      // DrumEvent
#include "policy/midi_score.hpp"
#include "policy/policy_config.hpp"

// 정책용 악보 스냅샷. 곡 시작 시 BehaviorPlanner 가 1회 발행한다.
//
// obs 의 next_hits (K x (M+3)) 를 만드는 것이 유일한 목적이다.
// 학습(robotic_drum_score.py::get_next_hits)과 같은 값이 나와야 하므로 규약 셋을 지킨다.
//
//   1) 양자화 — 학습의 rds 는 (T, M) 정수 시간 격자다(train_step_dt). 실기의 연속 시간을
//      같은 격자에 담아야 K개 윈도우에 같은 이벤트가 같은 순서로 들어간다.
//      부수 효과로 같은 칸에 든 두 타격은 하나의 이벤트로 묶인다(drum_mask 2비트).
//
//   2) 세기 — 이 클래스는 정규화된 u (0~1) 만 들고 있고, 변환은 로더가 한다.
//      입력 포맷이 둘이고 스케일이 다르기 때문이다:
//
//        MIDI (.mid)     u = velocity / 127            <- 학습의 원본 포맷. 변환 없음
//        실기 악보 (.txt) u = (v - 1) / 6               <- v 는 1~7 등급
//
//      후자의 근거: 학습이 reward.py 에서 u = midi/127 -> i = 1 + 6u 로 실기 등급을
//      복원해 쓴다(실기 state_motion_generator 공식을 그대로 이식). 그래서 역이 유일하다.
//      u <= 1 이므로 학습은 i > 7 을 표현할 수 없다 -> 로더가 v 를 [1,7] 로 clamp 한다.
//      변환 헬퍼는 아래 u_from_* 참조.
//
//   3) time_norm — 학습은 offset/(L-1) 로 정규화한다. 분모가 max_lookahead_s(2.0)가
//      아니라 (L-1) x step_dt = 1.98333 s 다. PolicyConfig::time_norm_denom() 참조.
//
// 밀도 경고: 학습 악보 생성기는 이벤트 간격을 min_gap = 2*hit_window_step+1 = 7 스텝
// (116.7 ms) 미만으로 만들지 않는다. hit_window(+-3 스텝)가 겹쳐 채점이 모호해지기
// 때문이다. 실기 악보에 더 촘촘한 구간이 있으면 정책이 본 적 없는 밀도이므로
// 성능 보장이 없다. 막지는 않고 로드 시 요약만 남긴다.
class PolicyScore {
public:
    static constexpr int NUM_DRUM = 8;      // obs 의 드럼 채널 수 (M)
    static constexpr int CHANNELS = NUM_DRUM + 3;   // multi-hot + time + valid + intensity
    static constexpr int V_MIN = 1;     // 실기 등급 하한
    static constexpr int V_MAX = 7;     // 실기 등급 상한 (학습이 u<=1 이라 표현 못하는 상한)

    // ===== 세기 변환 =====
    // 실기 악보의 1~7 등급 -> u. 범위를 벗어나면 clamp 하고 clamped 를 세운다.
    static double u_from_grade(int v, bool* clamped = nullptr);
    // MIDI velocity 1~127 -> u. 학습 원본 포맷이라 변환이 없다.
    static double u_from_midi(int velocity);

    struct Event {
        int    bin       = 0;      // lround(t_score / train_step_dt)
        uint8_t drum_mask = 0;     // 비트 d = obs 드럼 인덱스 d (0..7)
        float  u         = 0.0f;   // 정규화 세기 0~1. 동시타는 max
    };

    // 포맷 중립 입력. 로더가 u 로 변환해 넘긴다.
    struct RawHit {
        double t_score = 0.0;   // 악보 시간 [s]
        int    drum_id = 0;     // 실기 악기 id (1~8, 9=open hihat 는 5로 접힘)
        double u       = 0.0;   // 정규화 세기 0~1
    };

    // 포맷 중립 build. MIDI 로더도 이걸 쓴다.
    void build(const std::vector<RawHit>& hits,
               const PolicyConfig& cfg,
               const std::string& score_name = "");

    // 실기 .txt 악보(DrumEvent)용. events[0] 이 더미여도 무해하다
    // (note 가 전부 0 이면 이벤트로 세지 않는다).
    void build(const std::vector<DrumEvent>& events,
               const PolicyConfig& cfg,
               const std::string& score_name = "");

    // MIDI 용. DrumEvent 변환을 거치지 않고 원본 velocity(1~127)를 그대로 쓴다 —
    // 1~7 등급 왕복에서 정보가 깎이는 것을 피한다.
    void build(const MidiScore& midi,
               const PolicyConfig& cfg,
               const std::string& score_name = "");

    void clear();
    bool valid() const { return valid_; }
    size_t size() const { return events_.size(); }

    // 현재 악보 시간에서 다음 K개를 채운다. out 은 K*CHANNELS 개 float.
    //   speed_scale : ctx.play_speed_scale. 물리 시간은 배속되지 않으므로
    //                 time 채널은 실시간 기준으로 환산한다 (명세 02절).
    // 이벤트가 K개보다 적으면 나머지는 학습 기본값(time=1, valid=0)으로 채운다.
    void next_hits(double t_score_now, double speed_scale, float* out) const;

private:
    bool valid_ = false;
    std::vector<Event> events_;     // bin 오름차순, bin 중복 없음

    // build 시점의 설정 사본 (next_hits 가 재사용)
    double train_step_dt_    = 1.0 / 60.0;
    int    lookahead_steps_  = 120;
    double time_norm_denom_  = 119.0 / 60.0;
    int    num_hits_         = 6;
};


// PolicyScore 를 스레드 경계 너머로 공유하는 홀더.
//
// BehaviorPlanner(planner 스레드)가 곡 시작 시 1회 쓰고,
// PolicyRunner(정책 스레드)가 정책 주기마다 읽는다 (ROBOT::POLICY_DT_SECOND).
//
// 왜 뮤텍스 하나로 충분한가 — 흘러가는 데이터가 아니라 "최신 상태 하나"다.
// 정책은 현재 곡의 악보만 필요하고 같은 데이터를 여러 번 읽는다. 큐도 이중 버퍼도
// 필요 없다. 그런 구조는 스트림을 빠짐없이 전달할 때 쓰는 것이다.
//
// 왜 락이 실시간 예산에 문제가 안 되나 — 측정값 기준:
//   읽기 next_hits  0.05 us   (이벤트 1039개 악보, p99)
//   쓰기 build      65 us     (곡당 1회. 파일 파싱은 이 밖에서 이미 끝나 있다)
//   정책 주기       15,000 us
// 최악의 블로킹이 예산의 0.4% 다. recv(prio 30)가 planner 를 선점해 늘어나도 ~72us.
//
// 지키지 않으면 무슨 일이 나는가 — events_ 가 std::vector 라 build() 의 재할당 중에
// 읽으면 해제된 버퍼를 순회한다. 값이 틀리는 게 아니라 세그폴트다.
class PolicyScoreStore {
public:
    // ===== planner 전용 =====
    void publish(const std::vector<DrumEvent>& events, const PolicyConfig& cfg,
                 const std::string& name);
    void publish(const MidiScore& midi, const PolicyConfig& cfg,
                 const std::string& name);
    void clear();

    // ===== 정책 전용 =====
    // 악보가 없으면 false 를 반환하고 out 을 건드리지 않는다.
    bool next_hits(double t_score_now, double speed_scale, float* out) const;

    bool valid() const;

private:
    mutable std::mutex m_;
    PolicyScore score_;
};
