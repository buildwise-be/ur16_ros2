#include <memory>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "ur16e_unity_interfaces/srv/ur16e_mover_service.hpp"

using UR16eMoverService = ur16e_unity_interfaces::srv::UR16eMoverService;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class MoveUr16e : public rclcpp::Node {
public:
  MoveUr16e()
  : Node("move_ur16e") {
    // Create service
    service_ = this->create_service<UR16eMoverService>(
      "move_ur16e",
      std::bind(&MoveUr16e::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create action client
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    );

    // Initialize MoveGroup for planning
    planner_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "ur_manipulator"
    );

    RCLCPP_INFO(this->get_logger(), "move_ur16e node initialized");
  }

private:
  rclcpp::Service<UR16eMoverService>::SharedPtr service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> planner_;

  void handle_service(
    const std::shared_ptr<UR16eMoverService::Request> request,
    std::shared_ptr<UR16eMoverService::Response> response
  ) {
    RCLCPP_INFO(get_logger(), "Received move_ur16e service request");

    // Convert std::array<double,6> to std::vector<double>
    std::vector<double> joints_vec(request->joints.begin(), request->joints.end());
    auto trajectories = plan_move(joints_vec, request->target_pose);

    response->trajectories = trajectories;
    RCLCPP_INFO(get_logger(), "Service response sent with %zu trajectory(ies)", trajectories.size());
  }

  std::vector<moveit_msgs::msg::RobotTrajectory> plan_move(
    const std::vector<double>& joints,
    const geometry_msgs::msg::Pose& target_pose
  ) {
    // Wait for the action server
    RCLCPP_DEBUG(get_logger(), "Waiting for FollowJointTrajectory action server...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return {};
    }
    RCLCPP_DEBUG(get_logger(), "Action server available");

    // Set start state from provided joint positions
    moveit::core::RobotState start_state(*planner_->getCurrentState());
    start_state.setJointGroupPositions(planner_->getName(), joints);
    planner_->setStartState(start_state);
    RCLCPP_DEBUG(get_logger(), "Start state set from service request");

    // Set target pose
    planner_->setPoseTarget(target_pose);
    RCLCPP_DEBUG(get_logger(), "Target pose set");

    // Plan trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (planner_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      return {};
    }
    RCLCPP_INFO(get_logger(), "Planning successful");

    // Send the trajectory to the controller
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = plan.trajectory.joint_trajectory;
    RCLCPP_DEBUG(get_logger(), "Sending goal to action server");

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_options;
    send_options.result_callback = [](const GoalHandleFJT::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rclcpp::get_logger("move_ur16e"), "Execution succeeded");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("move_ur16e"), "Execution failed with code %d", static_cast<int>(result.code));
      }
    };
    action_client_->async_send_goal(goal_msg, send_options);
    RCLCPP_INFO(get_logger(), "Goal sent to controller");

    // Return the computed trajectory(s)
    return { plan.trajectory };
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveUr16e>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
