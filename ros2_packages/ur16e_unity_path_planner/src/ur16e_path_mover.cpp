#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <ur16e_unity_interfaces/srv/ur16e_path_plan.hpp>
#include <moveit/core/moveit_error_code.h>
#include <std_srvs/srv/set_bool.hpp>

using PathPlan   = ur16e_unity_interfaces::srv::UR16ePathPlan;

class PathMover : public rclcpp::Node, public std::enable_shared_from_this<PathMover>
{
public:
  PathMover(const rclcpp::NodeOptions &opts = {}) : Node("ur16e_path_mover", opts)
  {
    plan_srv_ = create_service<PathPlan>("ur16e_path_plan", std::bind(&PathMover::plan_cb, this, _1, _2));
    exec_srv_ = create_service<std_srvs::srv::SetBool>("ur16e_path_execute", std::bind(&PathMover::exec_cb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Services advertised (init deferred).");
  }

  void init_move_group()
  {
    static const std::string GROUP = "ur_manipulator";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), GROUP);
    RCLCPP_INFO(get_logger(), "MoveGroup initialized.");
  }

private:
  void plan_cb(const std::shared_ptr<PathPlan::Request> req, std::shared_ptr<PathPlan::Response> res)
  {
    if (req->target_poses.empty())
    {
      res->success = false;
      res->error_message = "No target poses provided.";
      res->planned_fraction = 0.0f;
      return;
    }

    std::vector<geometry_msgs::msg::Pose> wps = req->target_poses;
    if (req->closed_path && wps.front() != wps.back())
      wps.push_back(wps.front());

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(wps, 0.01, 0.0, traj);
    res->planned_fraction = fraction;

    if (fraction < 0.75)
    {
      res->success = false;
      res->error_message = "Cartesian planning incomplete; fraction=" + std::to_string(fraction);
      RCLCPP_WARN(get_logger(), "%s", res->error_message.c_str());
    }
    else
    {
      res->success = true;
      res->error_message = "";
      res->trajectory = traj;
      RCLCPP_INFO(get_logger(), "Cartesian path planned successfully (fraction=%.2f)", fraction);
    }

    last_traj_ = traj;
    has_traj_ = true;
  }

void exec_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (!has_traj_)
  {
    res->success = false;
    res->message = "No trajectory available.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = last_traj_;
  auto result = move_group_->execute(plan);

  res->success = static_cast<bool>(result);
  res->message = result ? "Execution started." : "Execution failed.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

  rclcpp::Service<PathPlan>::SharedPtr plan_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr exec_srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit_msgs::msg::RobotTrajectory last_traj_;
  bool has_traj_{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathMover>();
  node->init_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

