// policy_score / policy_config 확인.  make score-check
//
// 실기 악보를 로드해 양자화·세기 변환·next_hits 를 찍는다.
// 학습(get_next_hits) 규약과 눈으로 대조할 수 있게 K x (M+3) 을 그대로 출력한다.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "policy/midi_score.hpp"
#include "policy/policy_config.hpp"
#include "policy/policy_score.hpp"

// behavior_planner 의 악보 파싱을 그대로 옮긴 최소 버전 (t = beat*100/bpm + last_t)
static std::vector<DrumEvent> load_score(const std::string& path) {
    std::vector<DrumEvent> rds;
    std::ifstream f(path);
    if (!f.is_open()) { std::cerr << "열 수 없음: " << path << "\n"; return rds; }

    double bpm = 100.0, last_t = 0.0;
    std::string row;
    while (std::getline(f, row)) {
        std::istringstream iss(row);
        std::string item;
        std::vector<std::string> it;
        while (std::getline(iss, item, '\t')) it.push_back(item);
        if (it.empty() || it[0].empty()) continue;
        if (it[0] == "bpm") { if (it.size() > 1) bpm = std::stod(it[1]); continue; }
        if (it[0] == "end") break;
        if (it.size() < 8) continue;
        try {
            DrumEvent ev;
            ev.bar             = std::stoul(it[0]);
            ev.beat            = std::stod(it[1]);
            ev.note_num_R      = std::stoi(it[2]);
            ev.note_num_L      = std::stoi(it[3]);
            ev.velocity_R      = std::stoi(it[4]);
            ev.velocity_L      = std::stoi(it[5]);
            ev.is_kick         = std::stoi(it[6]) != 0;
            ev.is_closed_hihat = std::stoi(it[7]) != 0;
            ev.t = ev.beat * 100.0 / bpm + last_t;
            last_t = ev.t;
            rds.push_back(ev);
        } catch (...) { break; }
    }
    return rds;
}

int main(int argc, char** argv) {
    const std::string cfg_path = (argc > 1) ? argv[1]
        : "drumrobot_server/data/policy/obs_constants.json";
    const std::string score = (argc > 2) ? argv[2] : "BasicFillin";

    PolicyConfig cfg;
    if (!cfg.load(cfg_path)) return 1;
    cfg.print();

    // 세기 변환 표 — 두 포맷이 같은 u 로 수렴하는지 확인
    std::cout << "\n세기 변환\n  실기 등급 v -> u = (v-1)/6\n   ";
    for (int v = 0; v <= 8; ++v) {
        // 인자 평가 순서는 미정의다 — u 를 먼저 받고 나서 c 를 읽어야 한다
        bool c = false;
        const double u = PolicyScore::u_from_grade(v, &c);
        printf(" v%d=%.3f%s", v, u, c ? "*" : "");
    }
    printf("\n   (* = clamp)\n  MIDI velocity -> u = midi/127\n   ");
    for (int m : {1, 32, 64, 96, 127}) printf(" %d=%.3f", m, PolicyScore::u_from_midi(m));
    printf("\n");

    PolicyScore ps;
    const bool is_midi = score.size() > 4 && score.substr(score.size() - 4) == ".mid";

    if (is_midi) {
        MidiScore midi;
        if (!midi.load("drumrobot_server/data/midi/" + score)) return 1;
        ps.build(midi, cfg, score);
        auto evs = midi.to_drum_events();
        std::cout << "  to_drum_events(): " << evs.size() << "개 (페달·머리 생성기용)\n";
    } else {
        const std::string path = "drumrobot_server/data/scores/" + score + ".txt";
        auto rds = load_score(path);
        std::cout << "\n" << score << " — 악보 줄 " << rds.size() << "개\n";
        if (rds.empty()) return 1;
        ps.build(rds, cfg, score);
    }
    if (!ps.valid()) return 1;

    // next_hits 를 몇 시점에서 찍어본다
    std::vector<float> out(cfg.num_hits * PolicyScore::CHANNELS);
    for (double t : {0.0, 1.0, 2.4, 5.0}) {
        ps.next_hits(t, 1.0, out.data());
        printf("\nnext_hits(t_score=%.2f, speed=1.0)\n", t);
        printf("  k | drum multi-hot        | time   valid  u\n");
        for (int k = 0; k < cfg.num_hits; ++k) {
            const float* r = out.data() + k * PolicyScore::CHANNELS;
            printf("  %d |", k);
            for (int d = 0; d < PolicyScore::NUM_DRUM; ++d) printf(" %.0f", r[d]);
            printf("   | %.4f  %.0f     %.3f\n", r[8], r[9], r[10]);
        }
    }

    // 배속에서 time 채널이 줄어드는지 (이벤트가 있는 시점에서)
    printf("\n배속 확인 (t_score=2.40)\n");
    for (double sp : {1.0, 2.0}) {
        ps.next_hits(2.40, sp, out.data());
        printf("  speed %.1f  time: ", sp);
        for (int k = 0; k < cfg.num_hits; ++k)
            printf("%.4f ", out[k * PolicyScore::CHANNELS + 8]);
        printf("\n");
    }
    printf("  (speed 2.0 이 1.0의 절반이어야 정상 — 물리 시간은 배속되지 않음)\n");
    // ===== 타이밍 — 경합 창이 얼마나 되는지 =====
    printf("\n타이밍 (sizeof(Event) = %zu B, 이벤트 %zu개 = %.1f KB)\n",
           sizeof(PolicyScore::Event), ps.size(),
           ps.size() * sizeof(PolicyScore::Event) / 1024.0);

    {   // next_hits (읽기)
        const int N = 20000;
        std::vector<double> us(N);
        for (int i = 0; i < N; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            ps.next_hits(2.40 + 0.001 * i, 1.0, out.data());
            us[i] = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0).count();
        }
        std::sort(us.begin(), us.end());
        printf("  next_hits (읽기)  p50 %.3f us   p99 %.3f us   최대 %.3f us\n",
               us[N/2], us[N*99/100], us[N-1]);
    }
    {   // build (쓰기) — 파일 파싱 제외, 순수 build 만
        std::vector<PolicyScore::RawHit> raw;
        for (size_t i = 0; i < ps.size(); ++i)
            raw.push_back(PolicyScore::RawHit{i * 0.1, 1 + (int)(i % 8), 0.5});
        const int N = 200;
        std::vector<double> us(N);
        PolicyScore tmp;
        std::cout.setstate(std::ios::failbit);   // build 로그 억제
        for (int i = 0; i < N; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            tmp.build(raw, cfg, "");
            us[i] = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0).count();
        }
        std::cout.clear();
        std::sort(us.begin(), us.end());
        printf("  build (쓰기)      p50 %.1f us   p99 %.1f us   최대 %.1f us  (파일 파싱 제외)\n",
               us[N/2], us[N*99/100], us[N-1]);
    }
    return 0;
}
