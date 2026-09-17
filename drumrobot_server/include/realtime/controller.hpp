#pragma once
 
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <set>
#include <string>
 
#include "common/app_context.hpp"
#include "common/control_queue.hpp"
#include "policy/policy_target.hpp"
#include "common/robot_config.hpp"
#include "hardware/motor_codec.hpp"
#include "hardware/robot.hpp"
#include "util/audio_player.hpp"
#include "util/logger.hpp"
 
class Controller {
public:
    Controller(AppContext &ctxRef, ControlQueue &controlQueueRef, Robot &robotRef,
               AudioPlayer &audioRef, PolicyTarget &policyTargetRef);
    ~Controller();
 
    void send_loop();
    void recv_loop();
 
private:
    AppContext &ctx;
    ControlQueue &control_queue;
    Robot &robot;
 
    TMotorServoCodec t_codec;
    TMotorMITCodec   mit_codec;
    MaxonMotorCodec  m_codec;
 
    ControlSetPoint curr_point; // 현재 송신 중인 데이터 (5ms 주기)
    ControlSetPoint prev_point; // 맥슨 모터 보간 목표

    // pop 시점의 보정 전(raw) 머리 yaw.
    //
    // merge 의 허리 보정을 감산(-=)이 아니라 대입(= raw - waist)으로 하기 위한 원본.
    // 감산이면 언더플로 틱(큐가 비어 curr_point 를 재사용)에 merge 가 다시 돌 때
    // 이미 보정된 값에서 또 빼서 waist 만큼씩 드리프트한다. 대입은 몇 번 돌아도 같다.
    double raw_head_yaw = 0.0;

    // merge 가 마지막으로 쓴 허리각. 워치독 때 슬롯이 비어 있으면 이 값으로 보정한다.
    double last_waist_for_yaw = 0.0;
 
    // ===== SEND =====
    void send_task_1ms(int cnt);
    void send_task_5ms();

    bool all_tmotors_received();

    // 목표 13개가 전부 유한한가. NaN/Inf 면 그 관절을 로그하고 false.
    bool targets_finite(const ControlSetPoint &point);

    // MIT 제어 모드 진입 프레임 송신 (id 0~6). send_thread에서만 호출한다.
    // 게인 0 명령을 보내 MIT 피드백을 유도한다 (토크 0). 첫 수신 게이트 통과용.
    // 모든 TMotor 에 MIT 제어 모드 진입 프레임({FF..FC})을 보낸다.
    // 제어 시작 직전에 부른다 — 매뉴얼의 "enter before controlling" 지시를 따른다.
    void enter_mit_control_mode();

    void prime_mit_feedback();

    // 정책 슬롯의 최신값으로 팔 0~8을 덮어쓴다. 낡았으면 아무것도 하지 않는다.
    void merge_policy_target(uint64_t t);

    // 주기는 ROBOT::POLICY_TICK_STRIDE 하나가 권위다 (robot_config.hpp).
    static constexpr uint64_t POLICY_TICK_STRIDE = ROBOT::POLICY_TICK_STRIDE;
    // 정책이 3주기 연속 못 내면 실패로 본다. 주기를 바꿔도 "3주기"가 유지된다.
    static constexpr uint64_t WATCHDOG_TICKS     = 3 * POLICY_TICK_STRIDE;
    void tmotor_send_task(const ControlSetPoint &point);
    void maxon_motor_send_task(const ControlSetPoint &point);
    void dynamixel_send_task(const ControlSetPoint &point);

    void set_maxon_mode(std::shared_ptr<MaxonMotor> &maxon, ControlMode target_mode);
    // policy_owns_arm: 이 목표를 실은 setpoint 의 라벨. ctx.policy_active 를 소비
    // 시점에 직접 읽으면 안 된다 — 곡 종료 후 큐에 남은 정책 소유 프레임(<=100ms)에
    // 중력 보상이 잘못 켜진다 (control_queue.hpp 의 시차 규칙 참조).
    double cal_torque(std::shared_ptr<MaxonMotor> &maxon, double target_position,
                      bool policy_owns_arm);

    const double STICK_LEN_M   = 0.121;
    const double STICK_MASS_KG = 0.0845;

    const double POS_DIFF_LIMIT = 30.0f * M_PI / 180.0f; // 30도
 
    // ===== RECV =====
    void publish_joint_snapshot();  // 팔 9관절 일괄 발행 (함정 4)

    void read_frames(); // 소켓에서 프레임을 논블록으로 읽어 tempFrames 에 누적
    void distribute_frames();   // can frame을 파싱해 각 모터 상태 업데이트
 
    bool safety_check_recv_tmotor(std::shared_ptr<TMotor> &motor);
    bool safety_check_recv_maxon(std::shared_ptr<MaxonMotor> &motor);
 
    // 소켓별 수신 프레임 임시 버퍼  (socket_fd, frames)
    std::map<int, std::vector<struct can_frame>> temp_frames;

    // ===== log =====
    Logger motor_log;

    // ===== audio =====
    AudioPlayer &audio_player;

    // ===== policy =====
    PolicyTarget &policy_target;
};