#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>

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
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = static_cast<bool>(move_group_->plan(my_plan));

    if (!success)
    {
      std::string err = "MoveIt plan() failed.";
      RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
      response->success = false;
      response->error_message = err;
      return;
    }

    last_trajectory_ = my_plan.trajectory;
    has_trajectory_ = true;

    response->success = true;
    response->error_message = "";
    response->trajectory = my_plan.trajectory;

    RCLCPP_INFO(this->get_logger(), "Planning successful. Trajectory stored.");
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