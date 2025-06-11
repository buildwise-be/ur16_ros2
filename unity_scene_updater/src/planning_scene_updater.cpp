#include <rclcpp/rclcpp.hpp>
#include "your_package/srv/add_primitive.hpp"
#include <moveit/planning_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using AddPrimitive = your_package::srv::AddPrimitive;

class PlanningSceneUpdater : public rclcpp::Node
{
public:
  PlanningSceneUpdater()
  : Node("planning_scene_updater")
  {
    service_ = create_service<AddPrimitive>(
      "add_primitive_object",
      std::bind(&PlanningSceneUpdater::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service 'add_primitive_object' ready");
  }

private:
  void handle_request(
    const std::shared_ptr<AddPrimitive::Request>  req,
    std::shared_ptr<AddPrimitive::Response>       resp)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject obj;

    // Use the Unity-generated GUID directly as the collision-object ID
    obj.id = req->id;
    obj.header.frame_id = "world";

    // ADD or REMOVE
    obj.operation = req->remove
      ? moveit_msgs::msg::CollisionObject::REMOVE
      : moveit_msgs::msg::CollisionObject::ADD;

    // Only BOX and PLANE supported
    shape_msgs::msg::SolidPrimitive prim;
    if (req->type == "BOX" || req->type == "PLANE") {
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      double thickness = (req->type == "PLANE") ? 0.001 : req->dimensions.z;
      prim.dimensions = {
        req->dimensions.x,
        req->dimensions.y,
        thickness
      };
    } else {
      resp->success = false;
      resp->message = "Unsupported primitive type: " + req->type;
      // Echo back the GUID and remove flag even on failure
      resp->id     = req->id;
      resp->remove = req->remove;
      return;
    }

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(req->pose);

    // Apply to planning scene
    psi.applyCollisionObject(obj);

    // Fill the response
    resp->success = true;
    resp->message = std::string(req->remove ? "Removed object " : "Added object ")
                    + req->id;
    resp->id     = req->id;
    resp->remove = req->remove;
  }

  rclcpp::Service<AddPrimitive>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneUpdater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

