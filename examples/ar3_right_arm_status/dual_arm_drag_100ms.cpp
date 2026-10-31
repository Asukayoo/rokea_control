/**
 * 双臂：手动 → 下电 → 关节拖拽 100ms（1 kHz 记 q/tau）→ 关拖拽 → 确认下电。
 * （珞石拖拽须在下电状态下 enableDrag；「上电」此处指接通控制器并完成准备。）
 *
 * 用法：./dual_arm_drag_100ms [duration_ms=100] [out_prefix]
 * 默认 IP：192.168.2.160 / .161，本机 192.168.2.100
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;

namespace {

constexpr const char *kLocal = "192.168.2.100";
constexpr const char *kIpA = "192.168.2.160";
constexpr const char *kIpB = "192.168.2.161";

void check(const char *step, const error_code &ec) {
  if (ec)
    throw std::runtime_error(std::string(step) + ": " + ec.message() + " (" +
                             std::to_string(ec.value()) + ")");
}

const char *power_name(PowerState ps) {
  switch (ps) {
    case PowerState::on:
      return "on";
    case PowerState::off:
      return "off";
    case PowerState::estop:
      return "estop";
    case PowerState::gstop:
      return "gstop";
    default:
      return "?";
  }
}

struct Sample {
  int64_t t_ns = 0;
  int power = -1;
  std::array<double, 7> q{};
  std::array<double, 7> tau{};
};

struct ArmLog {
  std::string ip;
  std::string type;
  std::string side;
  bool ok = false;
  std::string err;
  std::string power_before;
  std::string power_during;  // mid sample
  std::string power_after;
  std::vector<Sample> samples;
  std::vector<std::string> notes;
};

std::string side_from_type(const std::string &type) {
  if (type.find("0.7L") != std::string::npos) return "LEFT";
  if (type.find("0.7R") != std::string::npos) return "RIGHT";
  return "UNKNOWN";
}

ArmLog run_arm(const std::string &ip, int duration_ms) {
  ArmLog log;
  log.ip = ip;
  xMateErProRobot *rp = nullptr;
  try {
    xMateErProRobot robot(ip, kLocal);
    rp = &robot;
    error_code ec;

    auto info = robot.robotInfo(ec);
    check("robotInfo", ec);
    log.type = info.type;
    log.side = side_from_type(info.type);

    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    check("NrtCommand", ec);
    robot.setOperateMode(OperateMode::manual, ec);
    check("manual", ec);

    // 准备：先确保可上电读一次状态，再按规范下电进拖拽
    auto ps0 = robot.powerState(ec);
    check("powerState0", ec);
    if (ps0 == PowerState::estop || ps0 == PowerState::gstop)
      throw std::runtime_error(std::string("estop/gstop: ") + power_name(ps0));

    if (ps0 != PowerState::on) {
      robot.setPowerState(true, ec);
      if (ec) {
        log.notes.push_back(std::string("setPowerState(true) skip/fail: ") + ec.message());
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        log.notes.push_back("powered on briefly for precheck");
      }
    }
    auto q_on = robot.jointPos(ec);
    auto tau_on = robot.jointTorque(ec);
    check("precheck joint", ec);
    (void)q_on;
    (void)tau_on;

    robot.setPowerState(false, ec);
    check("setPowerState(false) before drag", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    auto ps = robot.powerState(ec);
    check("powerState before drag", ec);
    log.power_before = power_name(ps);
    if (ps != PowerState::off)
      throw std::runtime_error(std::string("need power off for drag, got ") + power_name(ps));

    robot.enableDrag(DragParameter::jointSpace, DragParameter::freely, ec,
                     /*enable_drag_button=*/true);
    check("enableDrag", ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const int n = std::max(1, duration_ms);  // ~1 kHz → 1 sample/ms
    log.samples.reserve(static_cast<size_t>(n));
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    const int64_t t0 =
        static_cast<int64_t>(next.tv_sec) * 1000000000LL + next.tv_nsec;

    for (int i = 0; i < n; ++i) {
      Sample s{};
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      s.t_ns = static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec - t0;
      auto psc = robot.powerState(ec);
      s.power = static_cast<int>(psc);
      auto tau = robot.jointTorque(ec);
      auto q = robot.jointPos(ec);
      if (ec) throw std::runtime_error(std::string("sample: ") + ec.message());
      for (int j = 0; j < 7; ++j) {
        s.tau[j] = tau[j];
        s.q[j] = q[j];
      }
      log.samples.push_back(s);

      next.tv_nsec += 1000000L;
      if (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        next.tv_sec += 1;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    if (!log.samples.empty())
      log.power_during = power_name(static_cast<PowerState>(log.samples[n / 2].power));

    robot.disableDrag(ec);
    if (ec) log.notes.push_back(std::string("disableDrag: ") + ec.message());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(OperateMode::manual, ec);
    robot.setPowerState(false, ec);
    if (ec) log.notes.push_back(std::string("final off: ") + ec.message());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ps = robot.powerState(ec);
    log.power_after = power_name(ps);

    robot.disconnectFromRobot(ec);
    log.ok = true;
  } catch (const std::exception &e) {
    log.err = e.what();
    if (rp) {
      error_code ec;
      try {
        rp->disableDrag(ec);
      } catch (...) {
      }
      rp->setPowerState(false, ec);
    }
  }
  return log;
}

void analyze(const ArmLog &log) {
  std::cout << "\n=== " << log.side << " (" << log.ip << ") type=" << log.type << " ===\n";
  if (!log.ok) {
    std::cout << "FAILED: " << log.err << "\n";
    return;
  }
  std::cout << "power: before=" << log.power_before << " during≈" << log.power_during
            << " after=" << log.power_after << "\n";
  for (const auto &n : log.notes) std::cout << "note: " << n << "\n";

  const auto &s = log.samples;
  std::cout << "samples=" << s.size() << "\n";
  if (s.size() < 2) {
    std::cout << "ANOMALY: too few samples\n";
    return;
  }

  std::array<double, 7> q0 = s.front().q, q1 = s.back().q;
  std::array<double, 7> tmin{}, tmax{}, tabs_max{};
  for (int j = 0; j < 7; ++j) {
    tmin[j] = tmax[j] = s.front().tau[j];
    tabs_max[j] = std::abs(s.front().tau[j]);
  }
  size_t nan_q = 0, nan_tau = 0;
  double dq_max = 0, dtau_max = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    for (int j = 0; j < 7; ++j) {
      if (!std::isfinite(s[i].q[j])) ++nan_q;
      if (!std::isfinite(s[i].tau[j])) ++nan_tau;
      tmin[j] = std::min(tmin[j], s[i].tau[j]);
      tmax[j] = std::max(tmax[j], s[i].tau[j]);
      tabs_max[j] = std::max(tabs_max[j], std::abs(s[i].tau[j]));
      if (i > 0) {
        dq_max = std::max(dq_max, std::abs(s[i].q[j] - s[i - 1].q[j]));
        dtau_max = std::max(dtau_max, std::abs(s[i].tau[j] - s[i - 1].tau[j]));
      }
    }
  }
  double q_travel = 0;
  for (int j = 0; j < 7; ++j) q_travel = std::max(q_travel, std::abs(q1[j] - q0[j]));

  double dt_sum = 0, dt_max = 0;
  for (size_t i = 1; i < s.size(); ++i) {
    const double dt = (s[i].t_ns - s[i - 1].t_ns) / 1e6;
    dt_sum += dt;
    dt_max = std::max(dt_max, dt);
  }

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "dt_avg=" << dt_sum / (s.size() - 1) << " ms  dt_max=" << dt_max << " ms\n";
  std::cout << "q_travel_max=" << q_travel << " rad  dq_step_max=" << dq_max << " rad\n";
  std::cout << "tau_pp(Nm):";
  for (int j = 0; j < 7; ++j) std::cout << " " << (tmax[j] - tmin[j]);
  std::cout << "\n|tau|_max:";
  for (int j = 0; j < 7; ++j) std::cout << " " << tabs_max[j];
  std::cout << "\ndtau_step_max=" << dtau_max << " Nm  nan_q=" << nan_q
            << " nan_tau=" << nan_tau << "\n";

  std::vector<std::string> anom;
  if (log.power_before != "off") anom.emplace_back("drag start power != off");
  if (log.power_after != "off") anom.emplace_back("final power != off");
  if (nan_q || nan_tau) anom.emplace_back("NaN in q/tau");
  if (dt_max > 5.0) anom.emplace_back("sample dt_max > 5ms");
  // 无人拖时 100ms 内关节几乎不动；若突变可能异常
  if (q_travel > 0.05) anom.emplace_back("q moved >0.05 rad in 100ms (drag/external?)");
  if (dq_max > 0.02) anom.emplace_back("single-step dq >0.02 rad");
  // AR3 关节力矩量级粗判
  for (int j = 0; j < 7; ++j) {
    if (tabs_max[j] > 80.0) {
      anom.push_back("axis" + std::to_string(j + 1) + " |tau|>80 Nm");
    }
  }

  if (anom.empty())
    std::cout << "ANALYSIS: OK (no hard anomaly)\n";
  else {
    std::cout << "ANALYSIS: ANOMALIES:\n";
    for (const auto &a : anom) std::cout << "  - " << a << "\n";
  }
}

void write_csv(const ArmLog &log, const std::string &path) {
  if (!log.ok || log.samples.empty()) return;
  std::ofstream ofs(path);
  ofs << "t_ms,power,tau1,tau2,tau3,tau4,tau5,tau6,tau7,q1,q2,q3,q4,q5,q6,q7\n";
  ofs << std::setprecision(9);
  for (const auto &s : log.samples) {
    ofs << (s.t_ns / 1e6) << "," << s.power;
    for (double v : s.tau) ofs << "," << v;
    for (double v : s.q) ofs << "," << v;
    ofs << "\n";
  }
  std::cout << "wrote " << path << "\n";
}

}  // namespace

int main(int argc, char **argv) {
  const int duration_ms = (argc >= 2) ? std::stoi(argv[1]) : 100;
  const std::string prefix =
      (argc >= 3) ? argv[2]
                  : "/home/yxc/rokea_control/examples/ar3_right_arm_status/drag_dual_100ms";

  std::cout << "Dual-arm drag " << duration_ms << " ms (manual+off+enableDrag)\n";
  // SDK spdlog 全局 logger 名冲突，双臂须串行 connect（勿并行 new robot）
  ArmLog a = run_arm(kIpA, duration_ms);
  ArmLog b = run_arm(kIpB, duration_ms);

  analyze(a);
  analyze(b);
  write_csv(a, prefix + "_" + a.side + "_" + a.ip + ".csv");
  write_csv(b, prefix + "_" + b.side + "_" + b.ip + ".csv");

  const bool ok = a.ok && b.ok;
  std::cout << "\n" << (ok ? "DONE\n" : "DONE_WITH_ERRORS\n");
  return ok ? 0 : 1;
}
