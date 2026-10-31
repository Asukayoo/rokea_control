/**
 * 双臂 1 kHz 状态环（标准用法，对齐 nimo_arm_gui / xCoreSDK example）：
 *   SCHED_FIFO + 绑核 + mlockall；RtCommand + startReceiveRobotState(1ms)
 *
 * 左右由控制器 robotInfo.type 识别（0.7L / 0.7R），IP 自动探测 .160 / .161。
 *
 * 用法：
 *   sudo ./dual_arm_rt_1khz [duration_s=5] [cpu_right=2] [cpu_left=4]
 * 只接一臂时自动单臂测（左默认核4，右默认核2）。
 * 或（免 sudo）：
 *   sudo setcap cap_sys_nice,cap_ipc_lock+ep ./dual_arm_rt_1khz
 *   ./dual_arm_rt_1khz
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <thread>
#include <time.h>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;
using namespace RtSupportedFields;

namespace {

constexpr const char *kLocal = "192.168.2.100";
constexpr const char *kIpA = "192.168.2.160";
constexpr const char *kIpB = "192.168.2.161";

enum class Side { Left, Right, Unknown };

Side side_from_type(const std::string &type) {
  if (type.find("0.7L") != std::string::npos || type.find("_07L") != std::string::npos)
    return Side::Left;
  if (type.find("0.7R") != std::string::npos || type.find("_07R") != std::string::npos)
    return Side::Right;
  return Side::Unknown;
}

const char *side_name(Side s) {
  switch (s) {
    case Side::Left:
      return "LEFT";
    case Side::Right:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

void apply_rt(int cpu, int prio) {
  prctl(PR_SET_TIMERSLACK, 1);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    std::cerr << "warn: mlockall: " << strerror(errno) << "\n";

  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
    throw std::runtime_error(std::string("affinity: ") + strerror(errno));

  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
    throw std::runtime_error(std::string("SCHED_FIFO: ") + strerror(errno) +
                             " (use sudo or setcap cap_sys_nice,cap_ipc_lock+ep)");
}

struct Probe {
  std::string ip;
  std::string type;
  Side side = Side::Unknown;
};

Probe probe(const std::string &ip) {
  Probe p;
  p.ip = ip;
  xMateErProRobot robot(ip, kLocal);
  error_code ec;
  auto info = robot.robotInfo(ec);
  if (ec) throw std::runtime_error(ip + " robotInfo: " + ec.message());
  p.type = info.type;
  p.side = side_from_type(p.type);
  robot.disconnectFromRobot(ec);
  return p;
}

struct Result {
  std::string tag;
  std::string type;
  std::string ip;
  int cpu = -1;
  bool ok = false;
  std::string err;
  size_t n = 0;
  size_t empty = 0;
  double dt_avg = 0, dt_p99 = 0, dt_max = 0;
  size_t over_1p2 = 0, over_2p0 = 0;
};

Result run_arm(const Probe &probe_in, int cpu, double duration_s, std::atomic<bool> *abort) {
  Result r;
  r.ip = probe_in.ip;
  r.type = probe_in.type;
  r.cpu = cpu;
  r.tag = std::string(side_name(probe_in.side)) + "(" + probe_in.ip + ")";
  xMateErProRobot *robot = nullptr;

  try {
    apply_rt(cpu, 80);  // 规范常用 50–80；避免长期顶 99 饿死 IRQ 线程

    robot = new xMateErProRobot(probe_in.ip, kLocal);
    error_code ec;

    std::cout << r.tag << " type=" << r.type << " cpu=" << cpu << "\n" << std::flush;

    robot->setOperateMode(OperateMode::automatic, ec);
    if (ec) throw std::runtime_error(std::string("automatic: ") + ec.message());
    robot->setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec) throw std::runtime_error(std::string("RtCommand: ") + ec.message());

    robot->startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m, tau_m});

    const int nmax = std::max(1, static_cast<int>(std::lround(duration_s * 1000.0)));
    std::vector<double> dts;
    dts.reserve(static_cast<size_t>(nmax));
    std::array<double, 7> q{}, tau{};
    int64_t prev_ns = 0;

    for (int i = 0; i < nmax && !abort->load(); ++i) {
      const unsigned len = robot->updateRobotState(std::chrono::milliseconds(1));
      if (len == 0) {
        ++r.empty;
        continue;
      }
      if (robot->getStateData(jointPos_m, q) != 0)
        throw std::runtime_error("getStateData jointPos_m failed");
      robot->getStateData(tau_m, tau);

      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      const int64_t ns =
          static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
      if (prev_ns > 0) dts.push_back((ns - prev_ns) / 1e6);
      prev_ns = ns;
    }

    robot->stopReceiveRobotState();
    robot->setMotionControlMode(MotionControlMode::Idle, ec);
    robot->setOperateMode(OperateMode::manual, ec);
    robot->disconnectFromRobot(ec);

    if (dts.empty()) throw std::runtime_error("no timed samples");
    auto sorted = dts;
    std::sort(sorted.begin(), sorted.end());
    r.n = dts.size();
    r.dt_avg = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
    r.dt_max = sorted.back();
    r.dt_p99 = sorted[static_cast<size_t>((sorted.size() - 1) * 0.99)];
    for (double d : dts) {
      if (d > 1.2) ++r.over_1p2;
      if (d > 2.0) ++r.over_2p0;
    }
    r.ok = true;
  } catch (const std::exception &e) {
    r.err = e.what();
    abort->store(true);
    if (robot) {
      error_code ec;
      try {
        robot->stopReceiveRobotState();
      } catch (...) {
      }
      robot->setMotionControlMode(MotionControlMode::Idle, ec);
      robot->setOperateMode(OperateMode::manual, ec);
      robot->disconnectFromRobot(ec);
    }
  }
  delete robot;
  return r;
}

void print_r(const Result &r) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "=== " << r.tag << " type=" << r.type << " cpu=" << r.cpu << " ===\n";
  if (!r.ok) {
    std::cout << "FAILED: " << r.err << "\n";
    return;
  }
  std::cout << "samples=" << r.n << " empty_timeouts=" << r.empty << "\n"
            << "dt_avg=" << r.dt_avg << " ms  p99=" << r.dt_p99 << " ms  max=" << r.dt_max
            << " ms\n"
            << ">1.2ms: " << r.over_1p2 << " (" << 100.0 * r.over_1p2 / r.n << "%)  "
            << ">2.0ms: " << r.over_2p0 << " (" << 100.0 * r.over_2p0 / r.n << "%)\n";
}

}  // namespace

int main(int argc, char **argv) {
  const double dur = (argc >= 2) ? std::stod(argv[1]) : 5.0;
  const int cpu_right = (argc >= 3) ? std::stoi(argv[2]) : 2;
  const int cpu_left = (argc >= 4) ? std::stoi(argv[3]) : 4;

  std::cout << "RT 1kHz (FIFO + affinity; type decides L/R)\n";

  std::vector<Probe> found;
  for (const char *ip : {kIpA, kIpB}) {
    try {
      Probe p = probe(ip);
      std::cout << "probe " << p.ip << " -> type=" << p.type
                << " side=" << side_name(p.side) << "\n";
      found.push_back(std::move(p));
    } catch (const std::exception &e) {
      std::cout << "probe " << ip << " skip: " << e.what() << "\n";
    }
  }
  if (found.empty()) {
    std::cerr << "no arm reachable\n";
    return 1;
  }

  auto cpu_for = [&](Side s) {
    return (s == Side::Left) ? cpu_left : cpu_right;
  };

  std::atomic<bool> abort{false};
  std::vector<Result> results(found.size());
  std::vector<std::thread> threads;
  threads.reserve(found.size());

  if (found.size() == 1) {
    const Probe &p = found[0];
    if (p.side == Side::Unknown) {
      std::cerr << "unknown L/R from type, refuse to bind\n";
      return 1;
    }
    const int cpu = cpu_for(p.side);
    std::cout << "single-arm " << side_name(p.side) << "(" << p.ip << ") -> cpu " << cpu
              << "\n";
    results[0] = run_arm(p, cpu, dur, &abort);
  } else {
    if (cpu_right == cpu_left) {
      std::cerr << "cpu_right and cpu_left must differ\n";
      return 2;
    }
    Probe *right = nullptr;
    Probe *left = nullptr;
    for (auto &p : found) {
      if (p.side == Side::Right) right = &p;
      if (p.side == Side::Left) left = &p;
    }
    if (!right || !left) {
      std::cerr << "cannot map unique L/R from controller type\n";
      return 1;
    }
    std::cout << "RIGHT(" << right->ip << ") -> cpu " << cpu_right << "\n"
              << "LEFT (" << left->ip << ") -> cpu " << cpu_left << "\n";
    threads.emplace_back([&] { results[0] = run_arm(*right, cpu_right, dur, &abort); });
    threads.emplace_back([&] { results[1] = run_arm(*left, cpu_left, dur, &abort); });
    for (auto &t : threads) t.join();
  }

  bool all_ok = true;
  bool pass = true;
  for (const auto &r : results) {
    print_r(r);
    all_ok = all_ok && r.ok;
    pass = pass && r.ok && r.dt_p99 <= 1.5 && r.over_2p0 == 0;
  }
  std::cout << (pass ? "PASS\n" : "CHECK\n");
  return all_ok ? 0 : 1;
}
