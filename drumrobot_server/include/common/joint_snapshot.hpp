#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "common/robot_config.hpp"

// 팔 9관절의 시간 정합 스냅샷.
//
// recv_loop은 100us마다 돌면서 프레임이 도착한 모터부터 순서 없이 갱신한다.
// 정책이 모터를 하나씩 순회해 읽으면 "관절 0은 t, 관절 5는 t+50us"인 뒤섞인 상태가
// obs로 들어간다. 그래서 distribute_frames() 끝에서 9관절을 한 번에 발행한다.
class JointSnapshot {
public:
    struct Data {
        std::array<double, JointID::NUM_ARM> q{};      // 관절각 [rad]
        std::array<double, JointID::NUM_ARM> qd{};     // 관절 각속도 [rad/s]
        // 손목(7, 8)은 Maxon이라 속도 피드백이 없다. 여기서는 0으로 두고
        // PolicyRunner가 자기 주기(POLICY_DT)로 유한차분해 채운다 — 100us 간격
        // 차분은 엔코더 양자화 노이즈가 커서 쓸 수 없다. t_pub이 그 차분의 기준.
        uint64_t t_pub_ns = 0;                         // steady_clock 발행 시각
        bool     valid    = false;
    };

    void publish(const Data &d) {
        seq_.fetch_add(1, std::memory_order_acquire);
        std::atomic_thread_fence(std::memory_order_release);
        data_ = d;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);
    }

    Data read() const {
        Data out;
        // do-while 에서 continue 는 루프 처음이 아니라 조건식으로 간다. for(;;) 로 쓴다.
        for (;;) {
            uint32_t s0 = seq_.load(std::memory_order_acquire);
            if (s0 & 1u) continue;
            std::atomic_thread_fence(std::memory_order_acquire);
            out = data_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (s0 == seq_.load(std::memory_order_acquire)) break;
        }
        return out;
    }

private:
    mutable std::atomic<uint32_t> seq_{0};
    Data data_;
};
