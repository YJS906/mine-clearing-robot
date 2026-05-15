#include <cmath>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
    lift_wait_sec_ = declare_parameter<double>("lift_wait_sec", 1.0);
    use_joint_lift_pose_ = declare_parameter<bool>("use_joint_lift_pose", true);
    lift_joint1_rad_ = declare_parameter<double>("lift_joint1_rad", -1.4835298642);
    lift_joint2_rad_ = declare_parameter<double>("lift_joint2_rad", -0.1919862177);
    lift_joint3_rad_ = declare_parameter<double>("lift_joint3_rad", -0.4712388980);
    lift_joint4_rad_ = declare_parameter<double>("lift_joint4_rad", 2.181661565);
    cartesian_eef_step_m_ = declare_parameter<double>("cartesian_eef_step_m", 0.01);
    cartesian_jump_threshold_ = declare_parameter<double>("cartesian_jump_threshold", 0.0);
    cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.9);
    assume_close_after_timeout_ = declare_parameter<bool>("assume_close_after_timeout", true);
    close_gripper_timeout_sec_ = declare_parameter<double>("close_gripper_timeout_sec", 3.0);
    use_custom_close_position_ = declare_parameter<bool>("use_custom_close_position", true);
    close_gripper_joint_name_ = declare_parameter<std::string>(
      "close_gripper_joint_name", "gripper_left_joint");
    close_gripper_joint_position_m_ = declare_parameter<double>(
      "close_gripper_joint_position_m", 0.0045);
    auto_execute_cooldown_sec_ = declare_parameter<double>("auto_execute_cooldown_sec", 10.0);
    max_target_radius_m_ = declare_parameter<double>("max_target_radius_m", 0.36);
    min_target_z_m_ = declare_parameter<double>("min_target_z_m", -0.20);
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

    return moveGripper(open_target) &&
           moveArm(pregrasp, "pregrasp") &&
           moveArm(grasp, "grasp") &&
           moveGripper(close_target, true) &&
           liftAfterGrasp();
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

  bool liftAfterGrasp()
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    if (lift_wait_sec_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(lift_wait_sec_));
    }

    if (use_joint_lift_pose_) {
      return moveArmToJointLiftPose();
    }

    arm_->setStartStateToCurrentState();
    const auto current_pose = arm_->getCurrentPose();
    geometry_msgs::msg::Pose lift_pose = current_pose.pose;
    lift_pose.position.z += lift_height_m_;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(
      waypoints,
      cartesian_eef_step_m_,
      cartesian_jump_threshold_,
      trajectory);

    if (fraction < cartesian_min_fraction_) {
      RCLCPP_WARN(
        get_logger(),
        "Post-grasp lift failed: Cartesian fraction %.3f is below threshold %.3f.",
        fraction,
        cartesian_min_fraction_);
      arm_->stop();
      arm_->clearPoseTargets();
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto executed = arm_->execute(plan);
    arm_->stop();
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for post-grasp lift.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Post-grasp lift succeeded. fraction=%.3f", fraction);
    return true;
  }

  bool moveArmToJointLiftPose()
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(
      std::map<std::string, double>{
        {"joint1", lift_joint1_rad_},
        {"joint2", lift_joint2_rad_},
        {"joint3", lift_joint3_rad_},
        {"joint4", lift_joint4_rad_}});

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Planning failed for post-grasp joint lift pose.");
      arm_->clearPoseTargets();
      return false;
    }

    const auto executed = arm_->execute(plan);
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for post-grasp joint lift pose.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Post-grasp joint lift succeeded: [%.3f, %.3f, %.3f, %.3f]",
      lift_joint1_rad_,
      lift_joint2_rad_,
      lift_joint3_rad_,
      lift_joint4_rad_);
    return true;
  }

  bool moveGripper(const std::string & named_target, const bool is_close = false)
  {
    if (!gripper_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for gripper is not initialized.");
      return false;
    }
    if (is_close && use_custom_close_position_) {
      gripper_->setJointValueTarget(
        std::map<std::string, double>{{close_gripper_joint_name_, close_gripper_joint_position_m_}});
    } else {
      gripper_->setNamedTarget(named_target);
    }
    if (is_close && assume_close_after_timeout_) {
      gripper_->asyncMove();
      std::this_thread::sleep_for(std::chrono::duration<double>(close_gripper_timeout_sec_));
      gripper_->stop();
      RCLCPP_INFO(
        get_logger(),
        "Assuming gripper close completed after %.2f seconds.",
        close_gripper_timeout_sec_);
      return true;
    }

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
  double lift_wait_sec_{1.0};
  bool use_joint_lift_pose_{true};
  double lift_joint1_rad_{-1.4835298642};
  double lift_joint2_rad_{-0.1919862177};
  double lift_joint3_rad_{-0.4712388980};
  double lift_joint4_rad_{2.181661565};
  double cartesian_eef_step_m_{0.01};
  double cartesian_jump_threshold_{0.0};
  double cartesian_min_fraction_{0.9};
  bool assume_close_after_timeout_{true};
  double close_gripper_timeout_sec_{3.0};
  bool use_custom_close_position_{true};
  std::string close_gripper_joint_name_{"gripper_left_joint"};
  double close_gripper_joint_position_m_{0.0045};
  double auto_execute_cooldown_sec_{10.0};
  double max_target_radius_m_{0.36};
  double min_target_z_m_{-0.20};
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
