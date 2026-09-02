#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "common/robot_config.hpp"

// 정책 출력의 "최신값 슬롯". 큐가 아니다.
//
// 정책을 ControlQueue에 넣으면 MotionPlanner가 미리 채워 둔 100ms 버퍼 뒤에 줄을 서게 되어
// 폐루프가 성립하지 않는다. 그래서 정책은 항상 최신값 하나만 덮어쓰고, send_loop이
// 자기 주기에 맞춰 그 시점의 값을 읽어 간다. 값을 흘려도(정책이 늦어도) 문제가 없고,
// 오래된 값을 읽는 것만 막으면 된다 — 그래서 tick을 함께 싣는다.
//
// seqlock: 쓰기는 한 스레드(policy_thread), 읽기도 한 스레드(send_thread)뿐이지만
// 뮤텍스를 쓰면 실시간 송신 루프가 정책 스레드에 블로킹될 수 있다.
class PolicyTarget {
public:
    struct Snapshot {
        std::array<double, JointID::NUM_ARM> q{};
        uint64_t tick  = 0;      // 이 값을 만들 때의 ctx.tick
        bool     valid = false;  // 한 번이라도 발행됐는지
    };

    void publish(const std::array<double, JointID::NUM_ARM> &q, uint64_t tick) {
        seq_.fetch_add(1, std::memory_order_acquire);        // 홀수 = 쓰는 중
        std::atomic_thread_fence(std::memory_order_release);

        q_     = q;
        tick_  = tick;
        valid_ = true;

        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);        // 짝수 = 쓰기 완료
    }

    Snapshot snapshot() const {
        Snapshot out;
        // do-while 에서 continue 는 루프 처음이 아니라 조건식으로 간다. for(;;) 로 쓴다.
        for (;;) {
            uint32_t s0 = seq_.load(std::memory_order_acquire);
            if (s0 & 1u) continue;                           // 쓰는 중이면 다시

            std::atomic_thread_fence(std::memory_order_acquire);
            out.q     = q_;
            out.tick  = tick_;
            out.valid = valid_;
            std::atomic_thread_fence(std::memory_order_acquire);

            if (s0 == seq_.load(std::memory_order_acquire)) break;
        }
        return out;
    }

private:
    mutable std::atomic<uint32_t> seq_{0};

    std::array<double, JointID::NUM_ARM> q_{};
    uint64_t tick_  = 0;
    bool     valid_ = false;
};
