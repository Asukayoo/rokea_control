/**
 * AR3 右臂（A-LAN1）连通性检查：只连接并读取状态，不运动、不上电。
 *
 * 默认：机器人 192.168.1.160，本机 192.168.1.100
 * 用法：./ar3_right_arm_status [robot_ip] [local_ip]
 */

#include <iostream>
#include <string>

#include "rokae/robot.h"

namespace {

void print_usage(const char *argv0) {
  std::cerr << "Usage: " << argv0 << " [robot_ip] [local_ip]\n"
            << "  Defaults: robot_ip=192.168.1.160  local_ip=192.168.1.100\n"
            << "  (A-LAN1 / right arm)\n";
}

}  // namespace

int main(int argc, char **argv) {
  if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
    print_usage(argv[0]);
    return 0;
  }

  const std::string robot_ip = (argc >= 2) ? argv[1] : "192.168.1.160";
  const std::string local_ip = (argc >= 3) ? argv[2] : "192.168.1.100";

  std::cout << "Connecting xMateErProRobot (7-DoF)\n"
            << "  robot_ip = " << robot_ip << "\n"
            << "  local_ip = " << local_ip << "\n";

  try {
    // AR3 为协作 7 轴
    rokae::xMateErProRobot robot(robot_ip, local_ip);
    error_code ec;

    const auto info = robot.robotInfo(ec);
    if (ec) {
      std::cerr << "robotInfo failed: " << ec.message() << " (code " << ec.value() << ")\n";
      return 1;
    }

    std::cout << "OK  sdkVersion   = " << robot.sdkVersion() << "\n"
              << "OK  controller   = " << info.version << "\n"
              << "OK  type         = " << info.type << "\n"
              << "OK  joint_num    = " << info.joint_num << "\n";

    const auto q = robot.jointPos(ec);
    if (ec) {
      std::cerr << "jointPos failed: " << ec.message() << " (code " << ec.value() << ")\n";
      return 1;
    }

    std::cout << "OK  jointPos [rad] =";
    for (double qi : q) {
      std::cout << " " << qi;
    }
    std::cout << "\n";

    const auto power = robot.powerState(ec);
    if (!ec) {
      std::cout << "OK  powerState   = " << static_cast<int>(power) << "\n";
    }
    const auto mode = robot.operateMode(ec);
    if (!ec) {
      std::cout << "OK  operateMode  = " << static_cast<int>(mode) << "\n";
    }
    const auto op = robot.operationState(ec);
    if (!ec) {
      std::cout << "OK  operationState = " << static_cast<int>(op) << "\n";
    }

    robot.disconnectFromRobot(ec);
    std::cout << "Disconnected.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }
}
