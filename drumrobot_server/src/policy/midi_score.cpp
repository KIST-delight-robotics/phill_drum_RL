#include "policy/midi_score.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

namespace {

// ===== GM 퍼커션 노트 -> obs 드럼 인덱스 =====
// 학습 note_to_drum_idx (robotic_drum_score.py:62) 를 그대로 옮겼다.
// 값은 0-based obs 드럼 인덱스이고, 실기 악기 id 는 idx + 1 이다.
//   0 snare  1 floor  2 mid  3 top  4 hihat  5 ride  6 crash_r  7 crash_l
const std::map<int, int> kNoteToDrumIdx = {
    {38, 0}, {40, 0}, {37, 0},              // Acoustic/Electric Snare, Side Stick
    {41, 1}, {43, 1},                       // Low/High Floor Tom
    {45, 2}, {47, 2},                       // Low Tom, Low-Mid Tom
    {48, 3}, {50, 3}, {22, 3}, {26, 3},     // Hi Mid/High Tom, Tom(22), Tom(26)
    {42, 4}, {46, 4},                       // Closed Hi Hat, Open Hi-Hat (학습이 이미 접는다)
    {51, 5}, {59, 5}, {53, 5},              // Ride Cymbal 1/2, Ride Bell
    {52, 6}, {55, 6},                       // Chinese Cymbal, Splash Cymbal -> 오른쪽 크래시
    {49, 7}, {57, 7},                       // Crash Cymbal 1/2 -> 왼쪽 크래시
};

constexpr int kKick1 = 35;   // Acoustic Bass Drum
constexpr int kKick2 = 36;   // Bass Drum 1
constexpr int kPedalHihat = 44;
constexpr int kUnknown58  = 58;   // 학습 pedal_instruments 에 포함 ("뭔지 모르겠음")

constexpr int  kDrumChannel  = 9;        // GM 드럼 채널 10 (0-based)
constexpr uint32_t kDefaultTempo = 500000;   // us/quarter = 120 BPM

// ===== 바이트 읽기 =====
class Reader {
public:
    Reader(const uint8_t* p, size_t n) : p_(p), n_(n) {}

    bool ok() const { return ok_; }
    size_t pos() const { return i_; }
    size_t left() const { return (i_ < n_) ? (n_ - i_) : 0; }
    void seek(size_t i) { i_ = i; }

    uint8_t u8() {
        if (i_ + 1 > n_) { ok_ = false; return 0; }
        return p_[i_++];
    }
    uint16_t u16() { return static_cast<uint16_t>((u8() << 8) | u8()); }
    uint32_t u32() {
        uint32_t v = u8(); v = (v << 8) | u8(); v = (v << 8) | u8(); v = (v << 8) | u8();
        return v;
    }
    // SMF variable-length quantity (7비트씩, MSB 가 continuation)
    uint32_t vlq() {
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k) {
            const uint8_t b = u8();
            v = (v << 7) | (b & 0x7F);
            if (!(b & 0x80)) return v;
        }
        ok_ = false;
        return v;
    }
    bool tag(const char* four) {
        if (i_ + 4 > n_) { ok_ = false; return false; }
        const bool m = (p_[i_] == (uint8_t)four[0] && p_[i_ + 1] == (uint8_t)four[1] &&
                        p_[i_ + 2] == (uint8_t)four[2] && p_[i_ + 3] == (uint8_t)four[3]);
        i_ += 4;
        return m;
    }

private:
    const uint8_t* p_;
    size_t n_;
    size_t i_ = 0;
    bool ok_ = true;
};

}  // namespace

int MidiScore::gm_note_to_drum_idx(int note) {
    const auto it = kNoteToDrumIdx.find(note);
    return (it == kNoteToDrumIdx.end()) ? -1 : it->second;
}

bool MidiScore::gm_note_is_kick(int note) {
    return note == kKick1 || note == kKick2;
}

bool MidiScore::gm_note_is_pedal_hihat(int note) {
    return note == kPedalHihat;
}

void MidiScore::clear() {
    valid_ = false;
    bpm_ = 120.0;
    hits_.clear();
}

bool MidiScore::load(const std::string& path) {
    clear();

    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
        std::cerr << "[MidiScore] 열 수 없습니다: " << path << "\n";
        return false;
    }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)),
                              std::istreambuf_iterator<char>());
    if (buf.size() < 14) {
        std::cerr << "[MidiScore] 너무 짧습니다: " << path << "\n";
        return false;
    }

    Reader r(buf.data(), buf.size());

    // ---- MThd ----
    if (!r.tag("MThd")) {
        std::cerr << "[MidiScore] MThd 헤더가 아닙니다: " << path << "\n";
        return false;
    }
    const uint32_t hdr_len = r.u32();
    const uint16_t format  = r.u16();
    const uint16_t ntrks   = r.u16();
    const int16_t  division = static_cast<int16_t>(r.u16());
    if (hdr_len > 6) r.seek(r.pos() + (hdr_len - 6));    // 확장 헤더 건너뜀

    if (division <= 0) {
        // 음수면 SMPTE 타임코드. 드럼 MIDI 에서는 보기 어렵고 변환식이 달라 지원하지 않는다.
        std::cerr << "[MidiScore] SMPTE 타임코드는 지원하지 않습니다 (division="
                  << division << ")\n";
        return false;
    }
    const double ticks_per_beat = static_cast<double>(division);

    // ---- 전체 트랙에서 첫 set_tempo 를 찾아 bpm 을 정한다 (학습 파서와 같은 순서) ----
    // 아래 본 파싱에서 트랙별로 tempo 를 다시 추적하므로, 여기서는 bpm_ 표시용이다.
    uint32_t first_tempo = kDefaultTempo;
    int tempo_change_count = 0;

    // ---- 트랙 파싱 ----
    for (uint16_t t = 0; t < ntrks && r.ok() && r.left() >= 8; ++t) {
        if (!r.tag("MTrk")) {
            std::cerr << "[MidiScore] MTrk 를 찾지 못했습니다 (트랙 " << t << ")\n";
            break;
        }
        const uint32_t trk_len = r.u32();
        const size_t   trk_end = r.pos() + trk_len;

        uint64_t ticks = 0;             // 트랙별로 0에서 시작 (학습 파서와 동일)
        uint32_t tempo = kDefaultTempo; // 트랙별 초기화 (학습 파서와 동일)
        uint8_t  running_status = 0;

        while (r.ok() && r.pos() < trk_end) {
            ticks += r.vlq();

            uint8_t status = r.u8();
            if (status < 0x80) {
                // running status — 방금 읽은 바이트는 첫 데이터 바이트다
                if (running_status == 0) break;
                r.seek(r.pos() - 1);
                status = running_status;
            } else if (status < 0xF0) {
                running_status = status;
            }

            if (status == 0xFF) {                   // meta
                const uint8_t type = r.u8();
                const uint32_t len = r.vlq();
                if (type == 0x51 && len == 3) {     // set_tempo
                    const uint32_t v = (static_cast<uint32_t>(r.u8()) << 16) |
                                       (static_cast<uint32_t>(r.u8()) << 8) |
                                        static_cast<uint32_t>(r.u8());
                    if (tempo_change_count == 0) first_tempo = v;
                    if (v != tempo) ++tempo_change_count;
                    tempo = v;
                } else {
                    r.seek(r.pos() + len);
                }
                if (type == 0x2F) break;            // end of track
                continue;
            }
            if (status == 0xF0 || status == 0xF7) { // sysex
                const uint32_t len = r.vlq();
                r.seek(r.pos() + len);
                continue;
            }

            const uint8_t kind    = status & 0xF0;
            const uint8_t channel = status & 0x0F;

            // 데이터 바이트 개수
            int nbytes = 2;
            if (kind == 0xC0 || kind == 0xD0) nbytes = 1;   // program change, channel pressure

            const uint8_t d1 = r.u8();
            const uint8_t d2 = (nbytes == 2) ? r.u8() : 0;

            if (kind == 0x90 && d2 > 0 && channel == kDrumChannel) {
                // 학습 파서와 같은 식: 누적 tick x 현재 tempo / (ticks_per_beat x 1e6)
                const double t_sec =
                    (static_cast<double>(ticks) * static_cast<double>(tempo)) /
                    (ticks_per_beat * 1'000'000.0);

                Hit h;
                h.t_sec    = t_sec;
                h.gm_note  = d1;
                h.velocity = d2;
                h.drum_idx = gm_note_to_drum_idx(d1);
                h.is_kick        = gm_note_is_kick(d1);
                h.is_pedal_hihat = gm_note_is_pedal_hihat(d1);
                hits_.push_back(h);
            }
        }

        r.seek(trk_end);    // 트랙 경계로 정렬 (파싱 중 어긋나도 다음 트랙을 읽는다)
    }

    // 멀티 트랙이면 시간순으로 정렬 (학습 파서의 events.sort())
    std::stable_sort(hits_.begin(), hits_.end(),
                     [](const Hit& a, const Hit& b) { return a.t_sec < b.t_sec; });

    bpm_ = 60'000'000.0 / static_cast<double>(first_tempo);
    valid_ = !hits_.empty();

    // ---- 요약 ----
    int mapped = 0, pedal = 0, unknown = 0;
    for (const Hit& h : hits_) {
        if (h.drum_idx >= 0) ++mapped;
        else if (h.is_kick || h.is_pedal_hihat || h.gm_note == kUnknown58) ++pedal;
        else ++unknown;
    }
    std::cout << "[MidiScore] " << path << "\n"
              << "    format " << format << ", 트랙 " << ntrks
              << ", division " << division << " tick/beat, bpm " << bpm_ << "\n"
              << "    타격 " << hits_.size() << "개 — 팔 " << mapped
              << ", 페달 " << pedal << ", 미매핑 " << unknown << "\n";

    if (tempo_change_count > 1) {
        std::cerr << "[MidiScore] set_tempo 가 " << tempo_change_count
                  << "회 바뀝니다. 학습 파서와 같은 단순식(누적 tick x 현재 tempo)을 쓰므로"
                  << " 템포 변화 구간의 시각이 어긋납니다.\n";
    }
    if (unknown > 0) {
        std::cerr << "[MidiScore] 매핑 없는 노트 " << unknown
                  << "개를 버립니다 (학습 note_to_drum_idx 에 없는 GM 노트).\n";
    }
    if (!valid_) {
        std::cerr << "[MidiScore] 채널 " << kDrumChannel
                  << "(GM 드럼)에 note_on 이 없습니다. 드럼 트랙이 맞는지 확인하세요.\n";
    }
    return valid_;
}

std::vector<DrumEvent> MidiScore::to_drum_events() const {
    std::vector<DrumEvent> out;
    if (!valid_) return out;

    // 같은 시각의 타격을 묶는다. MIDI 는 동시타가 tick 이 정확히 같게 들어온다.
    std::map<double, std::vector<const Hit*>> by_time;
    for (const Hit& h : hits_) by_time[h.t_sec].push_back(&h);

    double last_t = 0.0;
    int over_two = 0;

    for (const auto& [t, group] : by_time) {
        DrumEvent ev;
        ev.t = t;
        // beat 는 "100bpm 기준 초" 규약이다 (behavior_planner: t = beat * 100/bpm + last_t).
        ev.beat = (t - last_t) * bpm_ / 100.0;
        last_t = t;

        int arm = 0;
        for (const Hit* h : group) {
            if (h->is_kick)        ev.is_kick = true;
            if (h->is_pedal_hihat) ev.is_closed_hihat = true;
            if (h->drum_idx < 0)   continue;

            // MIDI 에는 손 지정이 없다. 정책 구간에서는 스케줄러가 팔을 정하므로
            // 팔 모션에 영향이 없고, 여기서는 앞에서부터 R, L 로 채운다.
            // velocity 는 실기 등급(1~7)으로 환산한다 — DrumEvent 를 쓰는 쪽
            // (state/head 생성기)이 그 스케일을 전제하기 때문이다.
            // 정책의 next_hits 는 이 변환을 거치지 않고 hits() 를 직접 쓴다.
            const int grade = 1 + static_cast<int>(std::lround(6.0 * h->velocity / 127.0));
            const int drum_id = h->drum_idx + 1;

            if (arm == 0)      { ev.note_num_R = drum_id; ev.velocity_R = grade; }
            else if (arm == 1) { ev.note_num_L = drum_id; ev.velocity_L = grade; }
            else               { ++over_two; }
            ++arm;
        }
        out.push_back(ev);
    }

    if (over_two > 0) {
        std::cerr << "[MidiScore] 같은 시각에 팔 타격이 3개 이상인 곳이 있어 "
                  << over_two << "타를 버렸습니다 (팔은 2개).\n"
                  << "            정책의 next_hits 는 버리지 않습니다 — DrumEvent 변환만의 제약입니다.\n";
    }
    return out;
}
