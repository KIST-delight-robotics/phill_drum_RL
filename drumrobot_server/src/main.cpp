#include <iostream>
#include <thread>
#include <pthread.h>
#include <map>
#include <string>
#include <memory>

#include "common/app_context.hpp"
#include "common/command_queue.hpp"
#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"
#include "hardware/robot.hpp"
#include "tcp/tcp_server.hpp"
#include "realtime/controller.hpp"
#include "trajectory/motion_planner.hpp"
#include "policy/policy_config.hpp"
#include "policy/policy_score.hpp"
#include "policy/policy_target.hpp"
#include "policy/policy_runner.hpp"

#define PORT 1951   // Phil Collins 출생연도

void set_priority(std::thread& t, int priority) {
    // 값이 클수록 더 높은 우선순위
    sched_param sch;
    sch.sched_priority = priority;
    if (pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "Failed to set priority " << priority << std::endl;
    }
}

int main(int argc, char* argv[]) {
    AppContext ctx;
    CommandQueue command_queue;
    ControlQueue control_queue;
    MotionQueue motion_queue;

    Robot robot;
    robot.initialize();
    ctx.tmotor_mit = robot.use_mit;     // motors.json 최상위 "tmotor_mit"
    ctx.policy_dry_run = robot.policy_dry_run;

    AudioPlayer audio_player;
    audio_player.initialize();
    
    // 정책 상수. 실패하면 정책 없이 진행한다 (기존 연주는 그대로 된다).
    PolicyConfig policy_cfg;
    if (policy_cfg.load("drumrobot_server/data/policy/obs_constants.json")) {
        policy_cfg.print();
    } else {
        std::cerr << "[main] 정책 상수를 읽지 못했습니다. 정책 없이 진행합니다.\n";
    }

    PolicyScoreStore policy_score;
    PolicyTarget policy_target;

    TcpServer server(ctx, PORT, command_queue);
    Controller controller(ctx, control_queue, robot, audio_player, policy_target);
    MotionPlanner motion_planner(ctx, command_queue, control_queue, motion_queue, robot, audio_player,
                                 policy_cfg, policy_score);

    // 정책 런너. 실패하면 ctx.policy_ready 가 false 로 남고, TrajectoryGenerator 가
    // 소유권을 넘기지 않으므로 기존 개루프 경로로 그대로 연주한다.
    PolicyRunner policy_runner(ctx, robot, policy_cfg, policy_score, policy_target);
    const bool policy_ok = policy_runner.initialize();
    if (!policy_ok) {
        std::cerr << "[main] 정책을 켜지 못했습니다. 개루프로 연주합니다.\n";
    }

    std::thread send_thread(&Controller::send_loop, &controller);
    std::thread recv_thread(&Controller::recv_loop, &controller);
    std::thread motion_planning_thread(&MotionPlanner::run, &motion_planner);
    std::thread tcp_server_thread(&TcpServer::run, &server);

    // 정책 스레드는 준비됐을 때만 띄운다.
    std::thread policy_thread;
    if (policy_ok) policy_thread = std::thread(&PolicyRunner::run, &policy_runner);

    set_priority(send_thread, 40);
    set_priority(recv_thread, 30);
    // 정책은 recv(30)보다 낮게 — 폭주해도 모터 상태 갱신이 굶으면 안 된다.
    // MotionPlanner(20)보다는 높게 — 궤적 생성은 무겁고 데드라인이 없다.
    if (policy_ok) set_priority(policy_thread, 25);
    set_priority(motion_planning_thread, 20);
    set_priority(tcp_server_thread, 10);

    send_thread.join();
    recv_thread.join();
    motion_planning_thread.join();
    tcp_server_thread.join();
    if (policy_thread.joinable()) {
        ctx.policy_cv.notify_all();   // wait_for 타임아웃을 기다리지 않도록
        policy_thread.join();
    }
    server.stop();

    return 0;
}