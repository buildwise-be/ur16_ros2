#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <ur16e_unity_interfaces/srv/ur16e_path_plan.hpp>
#include <ur16e_unity_interfaces/srv/ur16e_mover_service.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <robot_state/robot_state.h>

using PathPlanSrv   = ur16e_unity_interfaces::srv::UR16ePathPlan;
using MoverSrv      = ur16e_unity_interfaces::srv::UR16eMoverService;
using std::placeholders::_1;
using std::placeholders::_2;

class PathMover : public rclcpp::Node
{
public:
  PathMover(const rclcpp::NodeOptions &opts = {})
  : Node("ur16e_path_mover", opts)
  {
    // Advertise services
    path_plan_srv_   = create_service<PathPlanSrv>("ur16e_path_plan", std::bind(&PathMover::path_plan_cb, this, _1, _2));
    mover_srv_       = create_service<MoverSrv>("ur16e_mover_service", std::bind(&PathMover::mover_cb, this, _1, _2));
    exec_srv_        = create_service<std_srvs::srv::SetBool>("ur16e_path_execute", std::bind(&PathMover::exec_cb, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Services advertised: path_plan, mover_service, path_execute.");

    // Initialize MoveGroup
    static const std::string GROUP = "ur_manipulator";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), GROUP);
    // Optional: adjust to your actual robot workspace
    move_group_->setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
    RCLCPP_INFO(get_logger(), "MoveGroup initialized.");
  }

private:
  // Multi-waypoint Cartesian path planning
  void path_plan_cb(
    const std::shared_ptr<PathPlanSrv::Request> req,
    std::shared_ptr<PathPlanSrv::Response> res)
  {
    if (req->target_poses.empty()) {
      res->success = false;
      res->fail_msg = "No waypoints provided.";
      res->planned_fraction = 0.0;
      return;
    }

    move_group_->setStartStateToCurrentState();
    auto kinematic_state = move_group_->getCurrentState();
    const auto *jmg = kinematic_state->getJointModelGroup(move_group_->getName());

    // IK validation
    for (size_t i = 0; i < req->target_poses.size(); ++i) {
      const auto &p = req->target_poses[i];
      bool valid = kinematic_state->setFromIK(jmg, p, "tool0", 0.1);
      if (!valid) {
        res->success = false;
        res->fail_msg = "IK failed at waypoint " + std::to_string(i);
        return;
      }
    }

    // Compute Cartesian path with collision checking
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
      req->target_poses,
      /*eef_step=*/0.01,
      /*jump_threshold=*/0.0,
      traj,
      /*avoid_collisions=*/true);

    res->planned_fraction = fraction;
    if (fraction < 1.0) {
      res->success = false;
      res->fail_msg = "Incomplete Cartesian path: fraction=" + std::to_string(fraction);
      return;
    }

    res->trajectory = traj;
    res->success = true;
    RCLCPP_INFO(get_logger(), "Cartesian path planned (%.2f)", fraction);
  }

  // Single-target planning via global planner
  void mover_cb(
    const std::shared_ptr<MoverSrv::Request> req,
    std::shared_ptr<MoverSrv::Response> res)
  {
    // Build target
    const auto &goal = req->target_pose;
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = goal.position;
    goal_pose.orientation = goal.orientation;

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(goal_pose);
    move_group_->setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      res->success = true;
      res->trajectory = plan.trajectory_;
      RCLCPP_INFO(get_logger(), "Single-target plan SUCCESS");
    } else {
      res->success = false;
      res->fail_msg = "Global planner failed: " + std::to_string(static_cast<int>(result));
      RCLCPP_WARN(get_logger(), "%s", res->fail_msg.c_str());
    }
  }

  // Execute last planned trajectory
  void exec_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    // This example assumes the last trajectory is stored; implement storage similarly to snippet
    if (!last_traj_) {
      res->success = false;
      res->message = "No trajectory available.";
      return;
    }

    // Validate end state
    if (!move_group_->setJointValueTarget(last_traj_->joint_trajectory.points.back().positions)) {
      res->success = false;
      res->message = "Invalid end state.";
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
    exec_plan.trajectory_ = *last_traj_;
    bool ok = static_cast<bool>(move_group_->execute(exec_plan));

    res->success = ok;
    res->message = ok ? "Execution started." : "Execution failed.";
  }

  // Members
  rclcpp::Service<PathPlanSrv>::SharedPtr path_plan_srv_;
  rclcpp::Service<MoverSrv>::SharedPtr mover_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr exec_srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_msgs::msg::RobotTrajectory> last_traj_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathMover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}