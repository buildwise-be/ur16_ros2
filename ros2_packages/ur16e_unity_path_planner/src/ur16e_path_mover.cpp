#include <memory>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_interface/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "ur16e_path_plan/srv/ur16e_path_plan.hpp"
#include "std_srvs/srv/set_bool.hpp"

class UR16ePathMover
  : public rclcpp::Node
  , public std::enable_shared_from_this<UR16ePathMover>
{
public:
  UR16ePathMover()
  : Node("ur16e_path_mover",
         rclcpp::NodeOptions()
           .automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(get_logger(), "Initializing UR16ePathMover");

    // Set up PlanningSceneMonitor for collision checking
    planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), "robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    // Required MoveGroupInterface init :contentReference[oaicite:0]{index=0}
    move_group_ = std::make_shared<
      moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);

    // Services
    plan_service_ = create_service<ur16e_path_plan::srv::UR16ePathPlan>(
      "ur16e_path_plan",
      std::bind(&UR16ePathMover::planCallback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

    execute_service_ = create_service<std_srvs::srv::SetBool>(
      "ur16e_path_execute",
      std::bind(&UR16ePathMover::executeCallback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));
  }

private:
  using PlanSrv = ur16e_path_plan::srv::UR16ePathPlan;
  using SetBool  = std_srvs::srv::SetBool;

  void planCallback(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<PlanSrv::Request> request,
    std::shared_ptr<PlanSrv::Response> response)
  {
    std::ostringstream dbg;
    dbg << "Planning request: use_cartesian="
        << (request->use_cartesian ? "true" : "false")
        << ", closed_path="
        << (request->closed_path ? "true" : "false")
        << ", poses=" << request->target_poses.size() << "\n";
    RCLCPP_DEBUG(get_logger(), "%s", dbg.str().c_str());

    // Copy and optionally close the loop
    std::vector<geometry_msgs::msg::Pose> poses = request->target_poses;
    if (request->closed_path && !poses.empty()) {
      poses.push_back(poses.front());
      dbg << "  → Closed path: returning to start\n";
    }

    // Prepare
    const std::string group = move_group_->getName();
    const auto joint_model_group =
      move_group_->getJointModelGroup(group);
    robot_trajectory::RobotTrajectory combined_traj(
      move_group_->getRobotModel(), group);
    float planned_fraction = 1.0f;

    // Cartesian path planning :contentReference[oaicite:1]{index=1}
    if (request->use_cartesian) {
      dbg << "  → Computing Cartesian path with "
          << poses.size() << " waypoints\n";
      double fraction = move_group_->computeCartesianPath(
        poses, 0.01, 0.0, combined_traj);
      dbg << "  → Fraction achieved: " << fraction << "\n";
      response->planned_fraction = static_cast<float>(fraction);
      if (fraction < 1.0) {
        response->success   = false;
        response->fail_msg  = dbg.str();
        RCLCPP_ERROR(get_logger(),
                     "Cartesian planning failed (%.2f%%)", fraction * 100.0);
        return;
      }
    }
    else {
      // Sequential joint-space planning with IK & collision checks
      for (size_t i = 0; i < poses.size(); ++i) {
        const auto &pose = poses[i];
        dbg << "Pose[" << i << "]: IK check\n";
        auto state = move_group_->getCurrentState();
        if (!state->setFromIK(joint_model_group, pose, 5.0)) {
          dbg << "  ✗ IK failed at index " << i << "\n";
          response->success      = false;
          response->fail_msg     = dbg.str();
          response->planned_fraction = float(i) / poses.size();
          RCLCPP_ERROR(get_logger(), "IK failed at pose %zu", i);
          return;
        }
        dbg << "  ✓ IK OK\n";

        // Collision query
        if (planning_scene_monitor_
            ->getPlanningScene()
            ->isStateColliding(*state, group))
        {
          dbg << "  ✗ Collision at index " << i << "\n";
          response->success      = false;
          response->fail_msg     = dbg.str();
          response->planned_fraction = float(i) / poses.size();
          RCLCPP_ERROR(get_logger(), "Collision at pose %zu", i);
          return;
        }

        // Plan this segment
        dbg << "  → Planning segment " << i << "\n";
        move_group_->setStartState(*state);
        move_group_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ec = move_group_->plan(plan);
        if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
          dbg << "  ✗ Planning failed at index " << i
              << " (code=" << static_cast<int>(ec) << ")\n";
          response->success      = false;
          response->fail_msg     = dbg.str();
          response->planned_fraction = float(i) / poses.size();
          RCLCPP_ERROR(get_logger(),
                       "Joint planning failed at pose %zu", i);
          return;
        }
        dbg << "  ✓ Segment " << i << " planned, appending\n";
        combined_traj.append(plan.trajectory_);
      }
    }

    // Success: fill response
    combined_traj.getRobotTrajectoryMsg(response->trajectory);
    response->success         = true;
    response->fail_msg.clear();
    response->planned_fraction = planned_fraction;
    last_plan_.trajectory_     = combined_traj;

    RCLCPP_INFO(get_logger(), "Planning succeeded");
  }

  void executeCallback(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<SetBool::Request> /*unused*/,
    std::shared_ptr<SetBool::Response> response)
  {
    if (last_plan_.trajectory_.getWayPointCount() == 0) {
      RCLCPP_ERROR(get_logger(), "No plan to execute");
      response->success = false;
      response->message = "No plan available";
      return;
    }
    RCLCPP_INFO(get_logger(), "Executing planned trajectory");
    // Execute using MoveGroupInterface::execute() :contentReference[oaicite:2]{index=2}
    bool ok = move_group_->execute(last_plan_);
    response->success = ok;
    response->message = ok ? "Execution succeeded"
                           : "Execution failed";
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Execution error");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
    planning_scene_monitor_;
  rclcpp::Service<PlanSrv>::SharedPtr    plan_service_;
  rclcpp::Service<SetBool>::SharedPtr     execute_service_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR16ePathMover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}