/**
 * 官方流程：手动 + 下电 → 关节空间拖拽 → 1 kHz 记力矩，最长 10 s 或 Ctrl+C → 关拖拽并下电。
 *
 * 默认 A-LAN2：192.168.2.160 / 192.168.2.100
 * 用法：./drag_joint_torque_100ms [robot_ip] [local_ip] [csv_path] [duration_s]
 *
 * Ctrl+C：停止采样，disableDrag + setPowerState(false)，写 CSV 后退出。
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

#include "rokae/robot.h"

namespace {

std::atomic<bool> g_stop{false};

void on_sigint(int) { g_stop.store(true); }

void check(const char *step, const error_code &ec) {
  if (ec) {
    throw std::runtime_error(std::string(step) + ": " + ec.message() + " (" +
                             std::to_string(ec.value()) + ")");
  }
}

bool try_fifo(int prio) {
  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    std::cerr << "SCHED_FIFO prio=" << prio << " failed: " << std::strerror(errno)
              << " (run with sudo/chrt for RT)\n";
    return false;
  }
  std::cout << "SCHED_FIFO priority=" << prio << "\n";
  return true;
}

void sleep_until(timespec *ts) {
  ts->tv_nsec += 1000000L;
  if (ts->tv_nsec >= 1000000000L) {
    ts->tv_nsec -= 1000000000L;
    ts->tv_sec += 1;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, nullptr);
}

}  // namespace

int main(int argc, char **argv) {
  const std::string robot_ip = (argc >= 2) ? argv[1] : "192.168.2.160";
  const std::string local_ip = (argc >= 3) ? argv[2] : "192.168.2.100";
  const std::string csv_path =
      (argc >= 4) ? argv[3]
                  : "/home/yxc/rokea_control/examples/ar3_right_arm_status/drag_tau_10s.csv";
  const double duration_s = (argc >= 5) ? std::stod(argv[4]) : 10.0;
  const int max_samples = std::max(1, static_cast<int>(std::lround(duration_s * 1000.0)));

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  {
    std::ifstream f("/sys/kernel/realtime");
    std::string v;
    std::cout << "RT kernel: "
              << ((f && (f >> v) && v == "1") ? "yes" : "unknown/no") << "\n";
  }
  const bool rt = try_fifo(80);

  rokae::xMateErProRobot *robot_ptr = nullptr;
  try {
    rokae::xMateErProRobot robot(robot_ip, local_ip);
    robot_ptr = &robot;
    error_code ec;

    auto info = robot.robotInfo(ec);
    check("robotInfo", ec);
    std::cout << "type=" << info.type << " robot=" << robot_ip
              << " duration_s=" << duration_s << " (Ctrl+C to stop early)\n";

    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    check("NrtCommand", ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);
    check("manual", ec);
    robot.setPowerState(false, ec);
    check("setPowerState(false) before drag", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto ps = robot.powerState(ec);
    check("powerState", ec);
    if (ps == rokae::PowerState::estop || ps == rokae::PowerState::gstop) {
      throw std::runtime_error("estop/gstop; reset first");
    }
    std::cout << "pre-drag powerState=" << static_cast<int>(ps) << " (off=1)\n";

    robot.enableDrag(rokae::DragParameter::jointSpace, rokae::DragParameter::freely, ec,
                     /*enable_drag_button=*/true);
    check("enableDrag(jointSpace, freely, no-button)", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "drag ON, sampling @ 1 kHz up to " << max_samples << " samples...\n";

    struct Sample {
      int64_t t_ns;
      std::array<double, 7> tau;
      std::array<double, 7> q;
    };
    std::vector<Sample> samples;
    samples.reserve(static_cast<size_t>(max_samples));

    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    const int64_t t0_ns = static_cast<int64_t>(next.tv_sec) * 1000000000LL + next.tv_nsec;

    for (int i = 0; i < max_samples && !g_stop.load(); ++i) {
      Sample s{};
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      s.t_ns = static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec - t0_ns;

      auto tau = robot.jointTorque(ec);
      auto q = robot.jointPos(ec);
      if (ec) {
        std::cerr << "sample error at i=" << i << ": " << ec.message() << "\n";
        g_stop.store(true);
        break;
      }
      for (int j = 0; j < 7; ++j) {
        s.tau[j] = tau[j];
        s.q[j] = q[j];
      }
      samples.push_back(s);

      if ((i + 1) % 1000 == 0) {
        std::cout << "  ... " << (i + 1) << " samples (" << (i + 1) / 1000.0 << " s)\n"
                  << std::flush;
      }
      sleep_until(&next);
    }

    const bool interrupted = g_stop.load();
    std::cout << (interrupted ? "stopped by signal\n" : "duration reached\n");

    // 收尾：关拖拽 → 切回 NRT/手动 → 下电（Ctrl+C 也走这里）
    robot.disableDrag(ec);
    if (ec) std::cerr << "disableDrag: " << ec.message() << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (ec) std::cerr << "NrtCommand after drag: " << ec.message() << "\n";
    robot.setOperateMode(rokae::OperateMode::manual, ec);
    if (ec) std::cerr << "manual after drag: " << ec.message() << "\n";
    robot.setPowerState(false, ec);
    if (ec) std::cerr << "setPowerState(false): " << ec.message() << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ps = robot.powerState(ec);
    std::cout << "drag OFF, final powerState=" << static_cast<int>(ps) << " (1=off)\n";

    if (!samples.empty()) {
      std::array<double, 7> tmin{}, tmax{};
      for (int j = 0; j < 7; ++j) tmin[j] = tmax[j] = samples.front().tau[j];
      for (const auto &s : samples) {
        for (int j = 0; j < 7; ++j) {
          tmin[j] = std::min(tmin[j], s.tau[j]);
          tmax[j] = std::max(tmax[j], s.tau[j]);
        }
      }
      std::cout << std::fixed;
      std::cout << "n=" << samples.size() << "  tau peak-peak (Nm):";
      for (int j = 0; j < 7; ++j) std::cout << " " << (tmax[j] - tmin[j]);
      std::cout << "\n";

      double dt_sum = 0, dt_max = 0;
      int dt_n = 0;
      for (size_t i = 1; i < samples.size(); ++i) {
        const double dt_ms = (samples[i].t_ns - samples[i - 1].t_ns) / 1e6;
        dt_sum += dt_ms;
        dt_max = std::max(dt_max, dt_ms);
        ++dt_n;
      }
      std::cout << "dt avg=" << (dt_n ? dt_sum / dt_n : 0) << " ms max=" << dt_max
                << " ms  SCHED_FIFO=" << (rt ? "yes" : "no") << "\n";

      std::ofstream ofs(csv_path);
      ofs << "t_ms,tau1,tau2,tau3,tau4,tau5,tau6,tau7,q1,q2,q3,q4,q5,q6,q7\n";
      ofs << std::setprecision(9);
      for (const auto &s : samples) {
        ofs << (s.t_ns / 1e6);
        for (double v : s.tau) ofs << "," << v;
        for (double v : s.q) ofs << "," << v;
        ofs << "\n";
      }
      std::cout << "wrote " << csv_path << "\n";
    }

    robot.disconnectFromRobot(ec);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAILED: " << e.what() << "\n";
    if (robot_ptr) {
      error_code ec;
      robot_ptr->disableDrag(ec);
      robot_ptr->setPowerState(false, ec);
    }
    return 1;
  }
}
