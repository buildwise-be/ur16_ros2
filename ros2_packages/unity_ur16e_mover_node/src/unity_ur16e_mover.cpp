#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit/utils/moveit_error_code.h>

#include "ur16e_unity_interfaces/srv/ur16e_mover_service.hpp"

using UR16eService       = ur16e_unity_interfaces::srv::UR16eMoverService;
using ExecuteService     = std_srvs::srv::SetBool;
using RobotTrajectoryMsg = moveit_msgs::msg::RobotTrajectory;

using std::placeholders::_1;
using std::placeholders::_2;

class UnityUR16eMover : public rclcpp::Node
{
public:
  explicit UnityUR16eMover(const rclcpp::NodeOptions & opts = {})
  : Node("unity_ur16e_mover", opts)
  {
    // only make the services here — no shared_from_this() yet
    service_plan_ = this->create_service<UR16eService>(
      "ur16e_mover_service",
      std::bind(&UnityUR16eMover::handle_planning_service, this, _1, _2));
    service_execute_ = this->create_service<ExecuteService>(
      "ur16e_execute_trajectory",
      std::bind(&UnityUR16eMover::handle_execute_service, this, _1, _2));
    RCLCPP_INFO(get_logger(), "Services ready (MoveGroup deferred).");
  }

  /// Must be called on a fully constructed shared_ptr<...>
  void init_move_group()
  {
    const std::string PLANNING_GROUP = "ur_manipulator";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized.");
  }

private:
void handle_planning_service(
  const std::shared_ptr<UR16eService::Request> request,
  std::shared_ptr<UR16eService::Response> response)
{
  geometry_msgs::msg::Pose target_pose = request->target_pose;
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit::core::MoveItErrorCode error_code;

  if (request->use_cartesian)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    double eef_step = 0.01;  // resolution in meters
    double jump_threshold = 0.0;  // disable jump detection for now

    double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.75)
    {
      response->success = false;
      response->error_message = "Cartesian path planning incomplete. Fraction: " + std::to_string(fraction);
      reponse->planned_fraction = fraction;
      RCLCPP_WARN(this->get_logger(), "Cartesian path planning incomplete. Fraction: %.2f", fraction);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Cartesian path planning successful. Fraction: %.2f", fraction);
  }
  else
  {
    move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    error_code = move_group_->plan(my_plan);

    if (!error_code)
    {
      std::string reason = moveit::core::errorCodeToString(error_code);
      RCLCPP_WARN(get_logger(), "MoveIt plan() failed: %s (%d)", reason.c_str(), error_code.val);
      response->success = false;
      response->error_message = reason;
      reponse->planned_fraction = 0.0;
      return;
    }

    trajectory = my_plan.trajectory;
    RCLCPP_INFO(this->get_logger(), "Standard motion planning successful.");
  }

  last_trajectory_ = trajectory;
  has_trajectory_ = true;

  response->success = true;
  response->error_message = "";
  response->trajectory = trajectory;
  reponse->planned_fraction = 1.0;
}

  void handle_execute_service(
    const std::shared_ptr<ExecuteService::Request> /*request*/,
    std::shared_ptr<ExecuteService::Response> response)
  {
    if (!has_trajectory_)
    {
      std::string err = "No trajectory available for execution.";
      RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
      response->success = false;
      response->message = err;
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan_to_execute;
    plan_to_execute.trajectory = last_trajectory_;

    move_group_->execute(plan_to_execute);
    response->success = true;
    response->message = "Trajectory executed.";
    RCLCPP_INFO(this->get_logger(), "Stored trajectory executed.");
  }

  rclcpp::Service<UR16eService>::SharedPtr service_plan_;
  rclcpp::Service<ExecuteService>::SharedPtr service_execute_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  RobotTrajectoryMsg last_trajectory_;
  bool has_trajectory_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityUR16eMover>();
  node->init_move_group();            // <<-- now shared_from_this() works
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
