#include "policy/policy_score.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstdint>
#include <map>

namespace {

// 실기 악기 id -> obs 드럼 인덱스.
//   실기 id 1~8 은 학습 id 1~8 과 같고 obs 인덱스는 id-1 이다.
//   id 0 (bass) 은 페달이라 obs 8드럼에 없다.
//   id 9 (open hihat) 은 학습에 없어 5 (closed hihat) 로 접는다 (결정 02).
//     근거: 두 악기의 좌표가 XY·손목각 동일, Z 만 30mm 차이이고
//     open/closed 는 팔이 아니라 왼발 페달이 정한다. 페달은 정책 밖이라 표현력을 잃지 않는다.
constexpr int OPEN_HIHAT_ID   = 9;
constexpr int CLOSED_HIHAT_ID = 5;

int obs_drum_index(int real_id) {
    int id = (real_id == OPEN_HIHAT_ID) ? CLOSED_HIHAT_ID : real_id;
    if (id < 1 || id > PolicyScore::NUM_DRUM) return -1;    // 0(bass) 및 범위 밖
    return id - 1;
}

}  // namespace

double PolicyScore::u_from_grade(int v, bool* clamped) {
    const int c = std::clamp(v, V_MIN, V_MAX);
    if (clamped) *clamped = (c != v);
    return static_cast<double>(c - V_MIN) / static_cast<double>(V_MAX - V_MIN);   // (v-1)/6
}

double PolicyScore::u_from_midi(int velocity) {
    return std::clamp(static_cast<double>(velocity) / 127.0, 0.0, 1.0);
}

void PolicyScore::clear() {
    valid_ = false;
    events_.clear();
}

void PolicyScore::build(const std::vector<RawHit>& hits,
                        const PolicyConfig& cfg,
                        const std::string& score_name) {
    clear();

    if (!cfg.valid) {
        std::cerr << "[PolicyScore] PolicyConfig 가 유효하지 않습니다\n";
        return;
    }

    train_step_dt_   = cfg.train_step_dt;
    lookahead_steps_ = cfg.lookahead_steps();
    time_norm_denom_ = cfg.time_norm_denom();
    num_hits_        = cfg.num_hits;

    // ---- train_step_dt 격자로 양자화하고 같은 칸을 하나의 이벤트로 묶는다 ----
    // map 이 bin 오름차순을 보장한다.
    std::map<int, Event> by_bin;
    int folded_open_hihat = 0;
    int dropped = 0;

    for (const RawHit& h : hits) {
        const int d = obs_drum_index(h.drum_id);
        if (d < 0) { ++dropped; continue; }
        if (h.drum_id == OPEN_HIHAT_ID) ++folded_open_hihat;

        const int bin = static_cast<int>(std::lround(h.t_score / train_step_dt_));

        Event& e = by_bin[bin];
        e.bin = bin;
        e.drum_mask = static_cast<uint8_t>(e.drum_mask | (1u << d));
        // 동시타는 max — 학습의 fut.amax(dim=-1) 와 같다
        e.u = std::max(e.u, static_cast<float>(std::clamp(h.u, 0.0, 1.0)));
    }

    events_.reserve(by_bin.size());
    for (auto& [bin, e] : by_bin) events_.push_back(e);
    valid_ = !events_.empty();

    // ---- 요약 및 밀도 경고 ----
    const std::string name = score_name.empty() ? std::string("(이름 없음)") : score_name;
    std::cout << "[PolicyScore] " << name << " — 이벤트 " << events_.size() << "개";
    if (folded_open_hihat > 0) std::cout << ", open hihat " << folded_open_hihat << "타를 closed 로 접음";
    if (dropped > 0) std::cout << ", obs 밖 타격 " << dropped << "개 제외(페달 등)";
    std::cout << "\n";

    if (events_.size() >= 2) {
        // 학습 악보 생성기의 최소 간격. hit_window(+-hit_window_step)가 겹치지 않는 하한이다.
        const int min_gap = 2 * cfg.hit_window_step + 1;
        int tight = 0, worst = INT32_MAX;
        for (size_t i = 1; i < events_.size(); ++i) {
            const int gap = events_[i].bin - events_[i - 1].bin;
            if (gap < min_gap) { ++tight; worst = std::min(worst, gap); }
        }
        if (tight > 0) {
            std::cerr << "[PolicyScore] 밀도 경고 — 간격 < " << min_gap << "스텝("
                      << min_gap * train_step_dt_ * 1000.0 << "ms) 구간 " << tight << "개"
                      << ", 최소 " << worst << "스텝(" << worst * train_step_dt_ * 1000.0 << "ms)\n"
                      << "              학습은 이보다 촘촘한 배치를 만들지 않습니다. "
                      << "해당 구간은 정책이 본 적 없는 밀도이므로 성능 보장이 없습니다.\n";
        }
    }
}

void PolicyScore::build(const std::vector<DrumEvent>& events,
                        const PolicyConfig& cfg,
                        const std::string& score_name) {
    std::vector<RawHit> hits;
    hits.reserve(events.size() * 2);

    int clamped_count = 0;
    int worst_v = 0;

    for (const DrumEvent& ev : events) {
        // 오른팔 / 왼팔 각각 최대 1타. note 0 은 무타격.
        const int  note[2] = {ev.note_num_R, ev.note_num_L};
        const int  vel[2]  = {ev.velocity_R, ev.velocity_L};

        for (int k = 0; k < 2; ++k) {
            if (note[k] == 0) continue;

            bool clamped = false;
            const double u = u_from_grade(vel[k], &clamped);
            if (clamped) {
                ++clamped_count;
                if (std::abs(vel[k] - 4) > std::abs(worst_v - 4)) worst_v = vel[k];
            }
            hits.push_back(RawHit{ev.t, note[k], u});
        }
        // is_kick / is_closed_hihat 은 페달이고 obs 8드럼에 없다. 여기서 다루지 않는다.
    }

    if (clamped_count > 0) {
        std::cerr << "[PolicyScore] velocity clamp " << clamped_count << "타 (가장 벗어난 값 "
                  << worst_v << ", 유효 범위 " << V_MIN << "~" << V_MAX << ")\n"
                  << "              학습은 u<=1 이라 등급 " << V_MAX
                  << " 초과를 표현할 수 없습니다. 해당 타격의 세기가 잘립니다.\n";
    }

    build(hits, cfg, score_name);
}

void PolicyScore::build(const MidiScore& midi,
                        const PolicyConfig& cfg,
                        const std::string& score_name) {
    std::vector<RawHit> hits;
    hits.reserve(midi.hits().size());

    for (const MidiScore::Hit& h : midi.hits()) {
        if (h.drum_idx < 0) continue;       // 페달·미매핑은 obs 8드럼에 없다
        hits.push_back(RawHit{h.t_sec,
                              h.drum_idx + 1,               // obs 인덱스 -> 실기 악기 id
                              u_from_midi(h.velocity)});    // 학습 원본 스케일. 변환 없음
    }

    build(hits, cfg, score_name);
}

void PolicyScore::next_hits(double t_score_now, double speed_scale, float* out) const {
    const int K = num_hits_;
    const int M = NUM_DRUM;

    // ---- 학습 기본값으로 초기화 ----
    // targets 0, time 1.0, valid 0, intensity 0  (get_next_hits 의 버퍼 초기값)
    for (int k = 0; k < K; ++k) {
        float* row = out + k * CHANNELS;
        for (int d = 0; d < M; ++d) row[d] = 0.0f;
        row[M]     = 1.0f;      // time
        row[M + 1] = 0.0f;      // valid
        row[M + 2] = 0.0f;      // intensity
    }

    if (!valid_) return;

    const double scale = (speed_scale > 1e-6) ? speed_scale : 1.0;
    const int now_bin = static_cast<int>(std::lround(t_score_now / train_step_dt_));

    // 윈도우: offset 0 포함, L 스텝. 학습의 offsets = arange(L) 와 같다.
    const int last_bin = now_bin + lookahead_steps_ - 1;

    // events_ 는 bin 오름차순이므로 now_bin 이상인 첫 원소부터 K개를 취한다.
    auto it = std::lower_bound(events_.begin(), events_.end(), now_bin,
                               [](const Event& e, int b) { return e.bin < b; });

    int filled = 0;
    for (; it != events_.end() && filled < K; ++it) {
        if (it->bin > last_bin) break;

        float* row = out + filled * CHANNELS;

        for (int d = 0; d < M; ++d) {
            row[d] = (it->drum_mask & (1u << d)) ? 1.0f : 0.0f;
        }

        // time_norm — 물리 시간은 배속되지 않으므로 실시간으로 환산한다 (명세 02절).
        // 분모는 (L-1) x step_dt 다. max_lookahead_s 가 아니다.
        const double t_to_hit_score = (it->bin - now_bin) * train_step_dt_;
        const double t_to_hit_real  = t_to_hit_score / scale;
        row[M]     = static_cast<float>(std::clamp(t_to_hit_real / time_norm_denom_, 0.0, 1.0));
        row[M + 1] = 1.0f;
        row[M + 2] = it->u;

        ++filled;
    }
}


// ===== PolicyScoreStore =====

void PolicyScoreStore::publish(const std::vector<DrumEvent>& events,
                               const PolicyConfig& cfg, const std::string& name) {
    std::lock_guard<std::mutex> lk(m_);
    score_.build(events, cfg, name);
}

void PolicyScoreStore::publish(const MidiScore& midi,
                               const PolicyConfig& cfg, const std::string& name) {
    std::lock_guard<std::mutex> lk(m_);
    score_.build(midi, cfg, name);
}

void PolicyScoreStore::clear() {
    std::lock_guard<std::mutex> lk(m_);
    score_.clear();
}

bool PolicyScoreStore::next_hits(double t_score_now, double speed_scale, float* out) const {
    std::lock_guard<std::mutex> lk(m_);
    if (!score_.valid()) return false;
    score_.next_hits(t_score_now, speed_scale, out);
    return true;
}

bool PolicyScoreStore::valid() const {
    std::lock_guard<std::mutex> lk(m_);
    return score_.valid();
}
