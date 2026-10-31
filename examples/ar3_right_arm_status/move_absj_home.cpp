/**
 * 低速 MoveAbsJ 回关节全 0，然后下电。
 * 默认 A-LAN2 右臂：robot=192.168.2.160 local=192.168.2.100
 * 用法：./move_absj_home [robot_ip] [local_ip] [joint_speed_0to1]
 */

#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "rokae/robot.h"

namespace {

void check(const char *step, const error_code &ec) {
  if (ec) {
    throw std::runtime_error(std::string(step) + ": " + ec.message() + " (" +
                             std::to_string(ec.value()) + ")");
  }
}

void print_q(const char *tag, const std::array<double, 7> &q) {
  std::cout << tag;
  for (double v : q) {
    std::cout << " " << v;
  }
  std::cout << "\n";
}

bool wait_idle(rokae::xMateErProRobot &robot, std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    error_code ec;
    const auto st = robot.operationState(ec);
    if (!ec && (st == rokae::OperationState::idle ||
                st == rokae::OperationState::unknown)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

}  // namespace

int main(int argc, char **argv) {
  const std::string robot_ip = (argc >= 2) ? argv[1] : "192.168.2.160";
  const std::string local_ip = (argc >= 3) ? argv[2] : "192.168.2.100";
  const double joint_speed = (argc >= 4) ? std::stod(argv[3]) : 0.05;  // 5%

  std::cout << "MoveAbsJ -> zeros @ jointSpeed=" << joint_speed
            << "\n  robot=" << robot_ip << " local=" << local_ip << "\n";

  try {
    rokae::xMateErProRobot robot(robot_ip, local_ip);
    error_code ec;

    auto info = robot.robotInfo(ec);
    check("robotInfo", ec);
    std::cout << "type=" << info.type << " joints=" << info.joint_num << "\n";

    auto q0 = robot.jointPos(ec);
    check("jointPos", ec);
    print_q("q_before [rad]", q0);

    auto ps = robot.powerState(ec);
    check("powerState", ec);
    if (ps == rokae::PowerState::estop || ps == rokae::PowerState::gstop) {
      throw std::runtime_error("powerState is estop/gstop; release and reset first");
    }

    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    check("setMotionControlMode", ec);

    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    check("setOperateMode(automatic)", ec);

    robot.setPowerState(true, ec);
    check("setPowerState(true)", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "powered on\n";

    robot.setDefaultSpeed(30, ec);  // mm/s 档，配合低 jointSpeed
    check("setDefaultSpeed", ec);

    rokae::MoveAbsJCommand cmd(rokae::JointPosition({0, 0, 0, 0, 0, 0, 0}), 30);
    cmd.jointSpeed = joint_speed;

    std::cout << "executing MoveAbsJ to zeros...\n";
    robot.executeCommand({cmd}, ec);
    check("executeCommand(MoveAbsJ)", ec);

    if (!wait_idle(robot, std::chrono::seconds(180))) {
      robot.stop(ec);
      throw std::runtime_error("timeout waiting for idle after MoveAbsJ");
    }
    std::cout << "motion finished (idle)\n";

    auto q1 = robot.jointPos(ec);
    check("jointPos after", ec);
    print_q("q_after  [rad]", q1);

    robot.setPowerState(false, ec);
    check("setPowerState(false)", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    ps = robot.powerState(ec);
    check("powerState final", ec);
    std::cout << "powered off, powerState=" << static_cast<int>(ps) << " (1=off)\n";

    robot.disconnectFromRobot(ec);
    std::cout << "done\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAILED: " << e.what() << "\n";
    return 1;
  }
}
