#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ColorGraspMoveIt : public rclcpp::Node
{
public:
  explicit ColorGraspMoveIt(const rclcpp::NodeOptions & options)
  : Node("color_grasp_moveit", options)
  {
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/vision/target_pose");
    execute_on_target_ = declare_parameter<bool>("execute_on_target", false);
    approach_height_m_ = declare_parameter<double>("approach_height_m", 0.06);
    grasp_height_m_ = declare_parameter<double>("grasp_height_m", 0.015);
    lift_height_m_ = declare_parameter<double>("lift_height_m", 0.08);
    auto_execute_cooldown_sec_ = declare_parameter<double>("auto_execute_cooldown_sec", 10.0);
    max_target_radius_m_ = declare_parameter<double>("max_target_radius_m", 0.36);
    min_target_z_m_ = declare_parameter<double>("min_target_z_m", 0.01);
    max_target_z_m_ = declare_parameter<double>("max_target_z_m", 0.28);
    last_auto_execute_time_ = now() - rclcpp::Duration::from_seconds(auto_execute_cooldown_sec_);

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_ = *msg;
        have_target_ = true;
        if (execute_on_target_ && !executing_ && autoCooldownElapsed()) {
          pending_auto_execute_ = true;
        }
      });

    pick_service_ = create_service<std_srvs::srv::Trigger>(
      "pick_latest_target",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = pickLatestTarget();
        response->message = response->success ? "pick sequence finished" : "pick sequence failed";
      });

    auto_timer_ = create_wall_timer(
      std::chrono::milliseconds(300),
      [this]() {
        if (!pending_auto_execute_) {
          return;
        }
        pending_auto_execute_ = false;
        if (executing_ || !autoCooldownElapsed()) {
          return;
        }
        executing_ = true;
        const bool ok = pickLatestTarget();
        last_auto_execute_time_ = now();
        executing_ = false;
        if (!ok) {
          RCLCPP_WARN(get_logger(), "Automatic pick attempt failed.");
        }
      });
  }

  void initMoveGroups()
  {
    const auto arm_group = get_parameter_or<std::string>("arm_group", "arm");
    const auto gripper_group = get_parameter_or<std::string>("gripper_group", "gripper");
    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group);
    gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), gripper_group);
    arm_->setPlanningTime(5.0);
    arm_->setGoalPositionTolerance(get_parameter_or<double>("position_tolerance_m", 0.02));
    arm_->setGoalOrientationTolerance(get_parameter_or<double>("orientation_tolerance_rad", 0.25));
  }

private:
  bool pickLatestTarget()
  {
    geometry_msgs::msg::PoseStamped target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!have_target_) {
        RCLCPP_WARN(get_logger(), "No target pose received yet.");
        return false;
      }
      target = latest_target_;
    }

    if (!isTargetInsideWorkspace(target)) {
      RCLCPP_WARN(
        get_logger(), "Target rejected: x=%.3f y=%.3f z=%.3f",
        target.pose.position.x, target.pose.position.y, target.pose.position.z);
      return false;
    }

    const auto open_target = get_parameter_or<std::string>("open_gripper_target", "open");
    const auto close_target = get_parameter_or<std::string>("close_gripper_target", "close");

    geometry_msgs::msg::PoseStamped pregrasp = target;
    pregrasp.pose.position.z += approach_height_m_;
    geometry_msgs::msg::PoseStamped grasp = target;
    grasp.pose.position.z += grasp_height_m_;
    geometry_msgs::msg::PoseStamped lift = grasp;
    lift.pose.position.z += lift_height_m_;

    return moveGripper(open_target) &&
           moveArm(pregrasp, "pregrasp") &&
           moveArm(grasp, "grasp") &&
           moveGripper(close_target) &&
           moveArm(lift, "lift");
  }

  bool autoCooldownElapsed() const
  {
    return (now() - last_auto_execute_time_).seconds() >= auto_execute_cooldown_sec_;
  }

  bool isTargetInsideWorkspace(const geometry_msgs::msg::PoseStamped & pose) const
  {
    const auto x = pose.pose.position.x;
    const auto y = pose.pose.position.y;
    const auto z = pose.pose.position.z;
    const auto radius = std::hypot(x, y);
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           radius <= max_target_radius_m_ &&
           z >= min_target_z_m_ &&
           z <= max_target_z_m_;
  }

  bool moveArm(const geometry_msgs::msg::PoseStamped & target, const std::string & label)
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }
    arm_->setStartStateToCurrentState();
    arm_->setPositionTarget(
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Planning failed for %s.", label.c_str());
      arm_->clearPoseTargets();
      return false;
    }
    const auto executed = arm_->execute(plan);
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for %s.", label.c_str());
      return false;
    }
    return true;
  }

  bool moveGripper(const std::string & named_target)
  {
    if (!gripper_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for gripper is not initialized.");
      return false;
    }
    gripper_->setNamedTarget(named_target);
    const auto result = gripper_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Gripper move to '%s' failed.", named_target.c_str());
      return false;
    }
    return true;
  }

  std::string target_pose_topic_;
  bool execute_on_target_{false};
  bool have_target_{false};
  bool pending_auto_execute_{false};
  bool executing_{false};
  double approach_height_m_{0.06};
  double grasp_height_m_{0.015};
  double lift_height_m_{0.08};
  double auto_execute_cooldown_sec_{10.0};
  double max_target_radius_m_{0.36};
  double min_target_z_m_{0.01};
  double max_target_z_m_{0.28};
  rclcpp::Time last_auto_execute_time_;

  std::mutex target_mutex_;
  geometry_msgs::msg::PoseStamped latest_target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorGraspMoveIt>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->initMoveGroups();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
