/**
 * Dual-arm Rokae joint-space drag → ROS 2 JointState (Marvin / tj_control signs).
 *
 * Startup: connect both arms (NrtCommand / manual / power off), drag OFF until
 * `rokea/drag_bridge/enable` is true (mirrors Marvin servoj_teleop_enabled).
 * false → disableDrag + power off (keep TCP). Publish mapped q only while drag ON.
 *
 * Axis map (Rokea → Marvin): q_tj[i] = sign_flip[i] * q_rokea[i]
 * Default sign_flip flips joints 2/4/6 (1-based).
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rokae/robot.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace rokae;
using namespace std::chrono_literals;

namespace
{

enum class Side { Left, Right, Unknown };

Side SideFromType(const std::string & type)
{
  if (type.find("0.7L") != std::string::npos || type.find("_07L") != std::string::npos) {
    return Side::Left;
  }
  if (type.find("0.7R") != std::string::npos || type.find("_07R") != std::string::npos) {
    return Side::Right;
  }
  return Side::Unknown;
}

const char * SideName(Side s)
{
  switch (s) {
    case Side::Left: return "LEFT";
    case Side::Right: return "RIGHT";
    default: return "UNKNOWN";
  }
}

void Check(const char * step, const error_code & ec)
{
  if (ec) {
    throw std::runtime_error(
      std::string(step) + ": " + ec.message() + " (" + std::to_string(ec.value()) + ")");
  }
}

struct ArmHandle {
  std::unique_ptr<xMateErProRobot> robot;
  std::string ip;
  std::string type;
  Side side{Side::Unknown};
  bool drag_on{false};
  std::array<double, 7> q{};
  std::mutex mu;
};

class DragBridgeNode : public rclcpp::Node
{
public:
  DragBridgeNode()
  : Node("rokea_drag_bridge")
  {
    declare_parameter<double>("rate_hz", 50.0);
    declare_parameter<std::string>("local_ip", "192.168.2.100");
    declare_parameter<std::string>("ip_a", "192.168.2.160");
    declare_parameter<std::string>("ip_b", "192.168.2.161");
    declare_parameter<std::string>("left_topic", "arm/left/target_joints");
    declare_parameter<std::string>("right_topic", "arm/right/target_joints");
    declare_parameter<std::string>("status_topic", "rokea/drag_bridge/status");
    declare_parameter<std::string>("enable_topic", "rokea/drag_bridge/enable");
    declare_parameter<std::vector<double>>(
      "sign_flip", std::vector<double>{1, -1, 1, -1, 1, -1, 1});

    rate_hz_ = get_parameter("rate_hz").as_double();
    if (rate_hz_ <= 0.0) {rate_hz_ = 50.0;}
    local_ip_ = get_parameter("local_ip").as_string();
    ip_a_ = get_parameter("ip_a").as_string();
    ip_b_ = get_parameter("ip_b").as_string();
    const auto signs = get_parameter("sign_flip").as_double_array();
    for (size_t i = 0; i < 7; ++i) {
      sign_flip_[i] = (i < signs.size()) ? signs[i] : 1.0;
      if (std::abs(sign_flip_[i]) < 1e-9) {sign_flip_[i] = 1.0;}
    }

    const std::string left_topic = get_parameter("left_topic").as_string();
    const std::string right_topic = get_parameter("right_topic").as_string();
    const std::string status_topic = get_parameter("status_topic").as_string();
    const std::string enable_topic = get_parameter("enable_topic").as_string();

    pub_left_ = create_publisher<sensor_msgs::msg::JointState>(left_topic, rclcpp::QoS(1));
    pub_right_ = create_publisher<sensor_msgs::msg::JointState>(right_topic, rclcpp::QoS(1));
    pub_status_ = create_publisher<std_msgs::msg::Bool>(status_topic, rclcpp::QoS(1));
    // transient_local: pick up last enable from dual_arm_node if bridge starts later
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        desired_drag_.store(msg->data);
      });

    RCLCPP_INFO(
      get_logger(),
      "rokea_drag_bridge: local=%s ips=%s,%s rate=%.1f Hz topics=%s | %s",
      local_ip_.c_str(), ip_a_.c_str(), ip_b_.c_str(), rate_hz_, left_topic.c_str(),
      right_topic.c_str());
    RCLCPP_INFO(
      get_logger(), "sign_flip=[%.0f %.0f %.0f %.0f %.0f %.0f %.0f] (Rokea→Marvin)",
      sign_flip_[0], sign_flip_[1], sign_flip_[2], sign_flip_[3], sign_flip_[4], sign_flip_[5],
      sign_flip_[6]);
    RCLCPP_INFO(
      get_logger(),
      "gated by %s (true=enableDrag, false=disableDrag+power off); start with drag OFF",
      enable_topic.c_str());

    ConnectArmsPowerOff();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { OnTimer(); });
  }

  ~DragBridgeNode() override
  {
    ShutdownArms();
  }

private:
  void ConnectArmsPowerOff()
  {
    // Serial construct (SDK spdlog logger name conflict if parallel new).
    for (const std::string & ip : {ip_a_, ip_b_}) {
      auto arm = std::make_unique<ArmHandle>();
      arm->ip = ip;
      arm->robot = std::make_unique<xMateErProRobot>(ip, local_ip_);
      error_code ec;
      auto info = arm->robot->robotInfo(ec);
      Check((ip + " robotInfo").c_str(), ec);
      arm->type = info.type;
      arm->side = SideFromType(arm->type);
      if (arm->side == Side::Unknown) {
        throw std::runtime_error(ip + " unknown side from type=" + arm->type);
      }

      arm->robot->setMotionControlMode(MotionControlMode::NrtCommand, ec);
      Check((ip + " NrtCommand").c_str(), ec);
      arm->robot->setOperateMode(OperateMode::manual, ec);
      Check((ip + " manual").c_str(), ec);

      auto ps = arm->robot->powerState(ec);
      Check((ip + " powerState").c_str(), ec);
      if (ps == PowerState::estop || ps == PowerState::gstop) {
        throw std::runtime_error(ip + " estop/gstop");
      }
      if (ps != PowerState::on) {
        arm->robot->setPowerState(true, ec);
        if (!ec) {std::this_thread::sleep_for(200ms);}
      }
      (void)arm->robot->jointPos(ec);
      Check((ip + " precheck jointPos").c_str(), ec);

      arm->robot->setPowerState(false, ec);
      Check((ip + " power off").c_str(), ec);
      arm->drag_on = false;

      RCLCPP_INFO(
        get_logger(), "connected side=%s ip=%s type=%s (drag OFF, powered off)",
        SideName(arm->side), ip.c_str(), arm->type.c_str());

      if (arm->side == Side::Left) {
        left_ = std::move(arm);
      } else {
        right_ = std::move(arm);
      }
    }

    if (!left_ || !right_) {
      throw std::runtime_error("need both LEFT and RIGHT controllers");
    }
    connected_ = true;
  }

  void EnableDragOne(ArmHandle & arm)
  {
    error_code ec;
    auto ps = arm.robot->powerState(ec);
    Check((arm.ip + " powerState").c_str(), ec);
    if (ps == PowerState::estop || ps == PowerState::gstop) {
      throw std::runtime_error(arm.ip + " estop/gstop");
    }
    if (ps != PowerState::off) {
      arm.robot->setPowerState(false, ec);
      Check((arm.ip + " power off for drag").c_str(), ec);
      std::this_thread::sleep_for(200ms);
      ps = arm.robot->powerState(ec);
      Check((arm.ip + " powerState before drag").c_str(), ec);
      if (ps != PowerState::off) {
        throw std::runtime_error(arm.ip + " need power off for drag");
      }
    }
    arm.robot->enableDrag(
      DragParameter::jointSpace, DragParameter::freely, ec, /*enable_drag_button=*/true);
    Check((arm.ip + " enableDrag").c_str(), ec);
    arm.drag_on = true;
    std::this_thread::sleep_for(30ms);
    RCLCPP_INFO(
      get_logger(), "drag ON side=%s ip=%s", SideName(arm.side), arm.ip.c_str());
  }

  void DisableDragPowerOffOne(ArmHandle & arm)
  {
    error_code ec;
    try {
      if (arm.drag_on) {
        arm.robot->disableDrag(ec);
        arm.drag_on = false;
      }
      arm.robot->setMotionControlMode(MotionControlMode::NrtCommand, ec);
      arm.robot->setOperateMode(OperateMode::manual, ec);
      arm.robot->setPowerState(false, ec);
      RCLCPP_INFO(
        get_logger(), "drag OFF + power off side=%s ip=%s", SideName(arm.side),
        arm.ip.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "disable %s: %s", arm.ip.c_str(), e.what());
    }
  }

  void SyncDragToDesired()
  {
    if (!connected_ || !left_ || !right_) {return;}
    const bool want = desired_drag_.load();
    const bool have = left_->drag_on && right_->drag_on;
    if (want == have) {
      // One side out of sync (partial failure) → force both to want.
      if (left_->drag_on == right_->drag_on) {return;}
    }
    try {
      if (want) {
        if (!left_->drag_on) {EnableDragOne(*left_);}
        if (!right_->drag_on) {EnableDragOne(*right_);}
      } else {
        DisableDragPowerOffOne(*left_);
        DisableDragPowerOffOne(*right_);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "SyncDragToDesired: %s", e.what());
      try {
        DisableDragPowerOffOne(*left_);
        DisableDragPowerOffOne(*right_);
      } catch (...) {
      }
      desired_drag_.store(false);
    }
  }

  void ShutdownArms()
  {
    timer_.reset();
    auto stop_one = [this](std::unique_ptr<ArmHandle> & arm) {
        if (!arm || !arm->robot) {return;}
        error_code ec;
        try {
          if (arm->drag_on) {
            arm->robot->disableDrag(ec);
            arm->drag_on = false;
          }
          arm->robot->setMotionControlMode(MotionControlMode::NrtCommand, ec);
          arm->robot->setOperateMode(OperateMode::manual, ec);
          arm->robot->setPowerState(false, ec);
          arm->robot->disconnectFromRobot(ec);
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "shutdown %s: %s", arm->ip.c_str(), e.what());
        }
        arm.reset();
      };
    stop_one(left_);
    stop_one(right_);
    connected_ = false;
  }

  void MapAndFill(
    sensor_msgs::msg::JointState & msg, const std::array<double, 7> & q_rokea,
    const char * prefix) const
  {
    msg.name.resize(7);
    msg.position.resize(7);
    for (int i = 0; i < 7; ++i) {
      msg.name[static_cast<size_t>(i)] =
        std::string("Joint") + std::to_string(i + 1) + prefix;
      msg.position[static_cast<size_t>(i)] =
        sign_flip_[static_cast<size_t>(i)] * q_rokea[static_cast<size_t>(i)];
    }
  }

  bool SampleArm(ArmHandle & arm, std::array<double, 7> & out)
  {
    error_code ec;
    auto q = arm.robot->jointPos(ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s jointPos: %s", arm.ip.c_str(),
        ec.message().c_str());
      return false;
    }
    for (int i = 0; i < 7; ++i) {out[static_cast<size_t>(i)] = q[static_cast<size_t>(i)];}
    return true;
  }

  void OnTimer()
  {
    if (!connected_ || !left_ || !right_) {return;}

    SyncDragToDesired();

    // Publish q whenever connected (power-off OK) so ServoJ enable can MovJ-align
    // before/while drag turns on. Drag gate only affects enableDrag + status lamp.
    const bool drag_ok = left_->drag_on && right_->drag_on;
    {
      std::array<double, 7> q_l{}, q_r{};
      const bool ok_l = SampleArm(*left_, q_l);
      const bool ok_r = SampleArm(*right_, q_r);
      const auto stamp = now();

      if (ok_l) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "Base_L";
        MapAndFill(msg, q_l, "_L");
        pub_left_->publish(msg);
        std::lock_guard<std::mutex> lock(left_->mu);
        left_->q = q_l;
      }
      if (ok_r) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "Base_R";
        MapAndFill(msg, q_r, "_R");
        pub_right_->publish(msg);
        std::lock_guard<std::mutex> lock(right_->mu);
        right_->q = q_r;
      }
    }

    std_msgs::msg::Bool st;
    st.data = drag_ok;
    pub_status_->publish(st);
  }

  double rate_hz_{50.0};
  std::string local_ip_;
  std::string ip_a_;
  std::string ip_b_;
  std::array<double, 7> sign_flip_{{1, -1, 1, -1, 1, -1, 1}};
  bool connected_{false};
  std::atomic<bool> desired_drag_{false};

  std::unique_ptr<ArmHandle> left_;
  std::unique_ptr<ArmHandle> right_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

namespace fs = std::filesystem;

// xCoreSDK writes ./logs/global_logger_YYYY-MM-DD.log (cwd-relative). If that
// file was created as root, open() aborts the process. Prefer a user-owned dir.
static void EnsureWritableSdkLogCwd()
{
  const char * home = std::getenv("HOME");
  const fs::path base = home ? (fs::path(home) / ".cache" / "rokea_drag_bridge")
                             : fs::path("/tmp/rokea_drag_bridge");
  const fs::path logs = base / "logs";
  std::error_code ec;
  fs::create_directories(logs, ec);
  if (ec) {
    fprintf(stderr, "rokea_drag_bridge: cannot create %s: %s\n", logs.c_str(),
      ec.message().c_str());
    return;
  }
  fs::current_path(base, ec);
  if (ec) {
    fprintf(stderr, "rokea_drag_bridge: chdir %s failed: %s\n", base.c_str(),
      ec.message().c_str());
  }
}

int main(int argc, char ** argv)
{
  EnsureWritableSdkLogCwd();
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DragBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "rokea_drag_bridge fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
