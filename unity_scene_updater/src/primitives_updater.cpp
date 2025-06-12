#include "rclcpp/rclcpp.hpp"
#include "ur16e_unity_interfaces/srv/add_primitive.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using AddPrimitive = ur16e_unity_interfaces::srv::AddPrimitive;

class PrimitivesUpdater : public rclcpp::Node
{
public:
  PrimitivesUpdater()
  : Node("planning_scene_updater")
  {
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    service_ = this->create_service<AddPrimitive>(
      "add_primitive_object",
      std::bind(&PrimitivesUpdater::handle_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service 'add_primitive_object' is ready.");
  }

private:
  void handle_service(
    const std::shared_ptr<AddPrimitive::Request> request,
    std::shared_ptr<AddPrimitive::Response> response)
  {
    const std::string& object_id = request->id;

    if (request->remove)
    {
      planning_scene_interface_->removeCollisionObjects({object_id});
      response->success = true;
      response->message = "Object removed.";
      response->id = object_id;
      response->remove = true;
      RCLCPP_INFO(this->get_logger(), "Removed object with id: %s", object_id.c_str());
      return;
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = object_id;
    collision_object.header.frame_id = "world";  // Assuming the planning frame is "world"

    shape_msgs::msg::SolidPrimitive primitive;

    if (request->type == "BOX")
    {
      primitive.type = primitive.BOX;
      primitive.dimensions = {request->dimensions.x, request->dimensions.y, request->dimensions.z};
    }
    else if (request->type == "PLANE")
    {
      primitive.type = primitive.BOX;
      primitive.dimensions = {request->dimensions.x, request->dimensions.y, 0.01};  // Thin Z for plane
    }
    else
    {
      response->success = false;
      response->message = "Unsupported primitive type: " + request->type;
      response->id = object_id;
      response->remove = false;
      RCLCPP_WARN(this->get_logger(), "Invalid type: %s", request->type.c_str());
      return;
    }

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(request->pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObjects({collision_object});

    response->success = true;
    response->message = "Object added.";
    response->id = object_id;
    response->remove = false;
    RCLCPP_INFO(this->get_logger(), "Added object of type: %s with id: %s",
                request->type.c_str(), object_id.c_str());
  }

  rclcpp::Service<AddPrimitive>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitivesUpdater>());
  rclcpp::shutdown();
  return 0;
}