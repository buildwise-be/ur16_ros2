#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "ur16e_unity_interfaces/srv/ur16e_mover_service.hpp"

using UR16eService        = ur16e_unity_interfaces::srv::UR16eMoverService;
using RobotTrajectoryMsg  = moveit_msgs::msg::RobotTrajectory;

class UnityUR16eMover : public rclcpp::Node
{
public:
  UnityUR16eMover()
  : Node("unity_ur16e_mover")
  {
    // Create MoveGroupInterface for the UR16e planning group.
    // Adjust the planning group name if needed.
    const std::string PLANNING_GROUP = "ur_manipulator";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    // Optionally set planner parameters here (tolerances, planner, etc.)
    // move_group_->setPlannerId("RRTConnectkConfigDefault");
    // move_group_->setPlanningTime(5.0);

    // Create the service:
    service_ = this->create_service<UR16eService>(
      "ur16e_mover_service",
      std::bind(&UnityUR16eMover::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(),
                "Service 'ur16e_mover_service' ready to plan for UR16e.");
  }

private:
  void handle_service(
    const std::shared_ptr<UR16eService::Request>  request,
    std::shared_ptr<UR16eService::Response>       response)
  {
    // 1) Extract the target pose from the request
    geometry_msgs::msg::Pose target_pose = request->target_pose;

    RCLCPP_INFO(this->get_logger(),
                "Received planning request: [%.3f, %.3f, %.3f] (orientation: [%.3f, %.3f, %.3f, %.3f])",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);

    // 2) Set the pose target on MoveGroupInterface
    move_group_->setPoseTarget(target_pose);

    // 3) Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = static_cast<bool>(move_group_->plan(my_plan));

    if (!success)
    {
      std::string err = "MoveIt plan() returned false.";
      RCLCPP_WARN(this->get_logger(), err.c_str());
      response->success = false;
      response->error_message = err;
      return;
    }

    // 4) Copy the trajectory into the service response
    response->success = true;
    response->error_message = "";
    response->trajectory = my_plan.trajectory;  // RobotTrajectory

    RCLCPP_INFO(this->get_logger(),
                "Plan succeeded. Returning trajectory with %zu waypoints.",
                response->trajectory.joint_trajectory.points.size());
  }

  rclcpp::Service<UR16eService>::SharedPtr                           service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityUR16eMover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
