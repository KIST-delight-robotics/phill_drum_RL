#pragma once

#include <map>
#include <memory>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

#include "dynamixel_sdk.h"
#include "nlohmann/json.hpp"

#include "common/joint_snapshot.hpp"
#include "common/robot_config.hpp"
#include "hardware/can_interface.hpp"
#include "hardware/motor_codec.hpp"
#include "hardware/motor.hpp"

constexpr const char* DXL_PORT = "/dev/ttyUSB0";

class Robot {
public:
    Robot();
    ~Robot();

    void initialize();

    CanInterface can;
    std::map<int, std::shared_ptr<Motor>> motors;
    std::vector<std::shared_ptr<MaxonMotor>> virtual_maxon_motor;   // Sync 신호를 위한 가상 모터

    std::unique_ptr<dynamixel::GroupSyncWrite> dxl_sync_write;
    std::unique_ptr<dynamixel::GroupSyncRead>  dxl_sync_read;
    dynamixel::PortHandler *dxl_port = nullptr;
    dynamixel::PacketHandler *dxl_packet = nullptr;

    std::map<int, std::string> joint_names;

    // motors.json 최상위 "tmotor_mit". false 면 팔 TMotor가 서보 모드로 동작한다.
    bool use_mit = true;

    // 팔 9관절의 시간 정합 스냅샷. recv_loop이 발행하고 policy_thread가 읽는다.
    JointSnapshot joint_snapshot;

private:

    TMotorServoCodec t_codec;
    TMotorMITCodec   mit_codec;
    MaxonMotorCodec  m_codec;

    void init_motor_from_json();
    void set_motors_socket();
    void maxon_motor_setting();
    void set_zero_tmotor();
    void enter_mit_mode();   // TMotor를 MIT 제어 모드로. 명령이 없으면 토크도 없다
    void maxon_motor_enable();
    void set_maxon_motor_mode(const std::string& targetMode);
    void init_dynamixel();
    void set_dxl_latency(const std::string &dev_path, int latency_ms);
    void set_dxl_initial_pose();
};