#pragma once

#include <string>
#include <vector>

#include "common/motion_queue.hpp"      // DrumEvent

// Standard MIDI File 로더 + GM 퍼커션 매핑.
//
// 왜 필요한가: MIDI 는 학습의 원본 포맷이다. 학습은 MIDI 를 읽어 rds 를 만들고
// velocity 1~127 을 그대로 쓴다. 실기 .txt 악보는 velocity 를 1~7 등급으로 옮겨 담아
// 변환이 한 번 끼는데, MIDI 를 직접 받으면 그 변환이 사라진다.
//
// 학습 쪽 파서(robotic_drum_score.py::_read_midi_file)의 규약을 그대로 따른다:
//   - 채널 9 (GM 드럼 채널 10, 0-based) 의 note_on 만, velocity > 0 만
//   - time_sec = accumulated_ticks * tempo / (ticks_per_beat * 1e6)
//   - 트랙별로 tick 을 0에서 다시 세고, 마지막에 시간순 정렬
//   - tempo 는 트랙 진행 중 set_tempo 를 만나면 갱신 (단일 템포 파일 전제)
//
// 매핑은 학습 note_to_drum_idx 를 그대로 옮겼다. obs 드럼 인덱스(0~7)를 주므로
// 실기 악기 id 는 idx + 1 이다. 페달(베이스·페달하이햇)은 obs 8드럼에 없다.
class MidiScore {
public:
    struct Hit {
        double t_sec    = 0.0;
        int    gm_note  = 0;
        int    velocity = 0;    // 1~127 (MIDI 원본)
        int    drum_idx = -1;   // obs 드럼 인덱스 0~7. 페달이면 -1
        bool   is_kick  = false;    // GM 35, 36
        bool   is_pedal_hihat = false;  // GM 44
    };

    bool load(const std::string& path);
    void clear();

    bool valid() const { return valid_; }
    double bpm() const { return bpm_; }
    const std::vector<Hit>& hits() const { return hits_; }   // 시간 오름차순

    // 실기 파이프라인(페달 생성기, 궤적 구간 분할)이 쓸 DrumEvent 로 변환한다.
    //
    // 주의: 정책의 next_hits 는 이 변환을 거치지 않고 hits() 를 직접 쓴다.
    // velocity 1~127 -> 1~7 등급 왕복에서 정보가 깎이기 때문이다.
    //
    // 팔 배정(note_num_R/L)은 MIDI 에 정보가 없다. 정책 구간에서는 스케줄러가
    // 담당하므로 팔 모션에 영향이 없고, 여기서는 같은 시각의 타격을 앞에서부터
    // R, L 순으로 채운다. 3타 이상이면 앞의 둘만 쓰고 경고한다.
    std::vector<DrumEvent> to_drum_events() const;

    // GM 노트 -> obs 드럼 인덱스. 페달이거나 매핑 없으면 -1.
    static int gm_note_to_drum_idx(int note);
    static bool gm_note_is_kick(int note);
    static bool gm_note_is_pedal_hihat(int note);

private:
    bool   valid_ = false;
    double bpm_   = 120.0;
    std::vector<Hit> hits_;
};
