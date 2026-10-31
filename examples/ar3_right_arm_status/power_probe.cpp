/**
 * 探测 powerState：读状态 → 尝试上电 → 再读 → 尝试下电 → 再读。
 * 用法：./power_probe [robot_ip] [local_ip]
 */

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "rokae/robot.h"

namespace {

const char *power_name(rokae::PowerState s) {
  using PS = rokae::PowerState;
  switch (s) {
    case PS::on:
      return "on(0)";
    case PS::off:
      return "off(1)";
    case PS::estop:
      return "estop(2)";
    case PS::gstop:
      return "gstop(3)";
    default:
      return "unknown";
  }
}

void print_power(rokae::xMateErProRobot &robot, const char *tag) {
  error_code ec;
  const auto ps = robot.powerState(ec);
  if (ec) {
    std::cout << tag << " powerState FAILED: " << ec.message() << " (" << ec.value() << ")\n";
  } else {
    std::cout << tag << " powerState = " << power_name(ps) << "\n";
  }
}

}  // namespace

int main(int argc, char **argv) {
  const std::string robot_ip = (argc >= 2) ? argv[1] : "192.168.1.160";
  const std::string local_ip = (argc >= 3) ? argv[2] : "192.168.1.165";

  std::cout << "robot=" << robot_ip << " local=" << local_ip << "\n";

  try {
    rokae::xMateErProRobot robot(robot_ip, local_ip);
    error_code ec;

    auto info = robot.robotInfo(ec);
    if (!ec) {
      std::cout << "type=" << info.type << " joints=" << info.joint_num << "\n";
    }

    print_power(robot, "[1] initial");

    std::cout << "[2] setPowerState(true)...\n";
    robot.setPowerState(true, ec);
    if (ec) {
      std::cout << "    setPowerState(true) FAILED: " << ec.message() << " (" << ec.value()
                << ")\n";
    } else {
      std::cout << "    setPowerState(true) OK\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_power(robot, "[3] after power on");

    std::cout << "[4] setPowerState(false)...\n";
    robot.setPowerState(false, ec);
    if (ec) {
      std::cout << "    setPowerState(false) FAILED: " << ec.message() << " (" << ec.value()
                << ")\n";
    } else {
      std::cout << "    setPowerState(false) OK\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_power(robot, "[5] after power off");

    robot.disconnectFromRobot(ec);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }
}
