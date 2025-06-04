#include "moveit_object_attacher/object_attacher_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

void printObj(const rclcpp::Logger& logger, const moveit_msgs::msg::CollisionObject& object)
{
  RCLCPP_INFO(logger, "=== CollisionObject Received ===");
  RCLCPP_INFO(logger, "ID: %s", object.id.c_str());
  RCLCPP_INFO(logger, "Frame ID: %s", object.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Operation: %d", object.operation);

  // Primitive shapes
  for (size_t i = 0; i < object.primitives.size(); ++i)
  {
    const auto& prim = object.primitives[i];

    std::ostringstream dim_stream;
    for (size_t j = 0; j < prim.dimensions.size(); ++j)
    {
      dim_stream << prim.dimensions[j];
      if (j < prim.dimensions.size() - 1)
        dim_stream << ", ";
    }

    RCLCPP_INFO(logger, "Primitive %zu: type=%d, dimensions=[%s]",
                i, prim.type, dim_stream.str().c_str());

    if (i < object.primitive_poses.size())
    {
      const auto& pose = object.primitive_poses[i];
      RCLCPP_INFO(logger, "  Pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
  }

  // Meshes
  for (size_t i = 0; i < object.meshes.size(); ++i)
  {
    const auto& mesh = object.meshes[i];
    RCLCPP_INFO(logger, "Mesh %zu: %zu triangles, %zu vertices", i, mesh.triangles.size(), mesh.vertices.size());

    if (i < object.mesh_poses.size())
    {
      const auto& pose = object.mesh_poses[i];
      RCLCPP_INFO(logger, "  Pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
  }

  // Planes
  for (size_t i = 0; i < object.planes.size(); ++i)
  {
    const auto& plane = object.planes[i];
    RCLCPP_INFO(logger, "Plane %zu: coef=[%.3f, %.3f, %.3f, %.3f]",
                i, plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]);

    if (i < object.plane_poses.size())
    {
      const auto& pose = object.plane_poses[i];
      RCLCPP_INFO(logger, "  Pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
  }

  RCLCPP_INFO(logger, "===============================");
}

namespace moveit_object_attacher
{

ObjectAttacherNode::ObjectAttacherNode()
: Node("object_attacher_node"),
  move_group_(std::shared_ptr<rclcpp::Node>(this), this->declare_parameter<std::string>("group_name", "arm"))
{
  add_obj_srv_ = this->create_service<moveit_object_attacher::srv::AddObj>(
    "/add_scene_object",
    std::bind(&ObjectAttacherNode::addObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

  attach_obj_srv_ = this->create_service<moveit_object_attacher::srv::AttachObj>(
    "/attach_scene_object",
    std::bind(&ObjectAttacherNode::attachObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "ObjectAttacherNode started with group '%s'", move_group_.getName().c_str());
}

void ObjectAttacherNode::addObjectCallback(
  const std::shared_ptr<moveit_object_attacher::srv::AddObj::Request> request,
  std::shared_ptr<moveit_object_attacher::srv::AddObj::Response> response)
{
  const auto& object = request->object;
  const std::string& object_id = object.id;

  printObj(this->get_logger(),object);

  try
  {
    const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0); // Timeout massimo 5 secondi
    const rclcpp::Time start = this->now();
    bool success = false;

    if (request->add)
    {
      planning_scene_interface_.applyCollisionObject(object);
      RCLCPP_INFO(this->get_logger(), "Requested addition of object '%s'", object_id.c_str());

      while ((this->now() - start) < timeout)
      {
        auto known_objects = planning_scene_interface_.getKnownObjectNames();

        std::ostringstream obj_list;
        obj_list << "Known objects in the scene: [";
        for (size_t i = 0; i < known_objects.size(); ++i)
        {
          obj_list << known_objects[i];
          if (i < known_objects.size() - 1)
            obj_list << ", ";
        }
        obj_list << "]";
        RCLCPP_INFO(this->get_logger(), "%s", obj_list.str().c_str());

        if (std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end())
        {
          success = true;
          break;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }


      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Object '%s' successfully added", object_id.c_str());

        response->success = true;
        response->message = "Object added and verified";
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Timeout: object '%s' not found in scene", object_id.c_str());
        response->success = false;
        response->message = "Object addition timed out";
      }
    }
    else
    {
      planning_scene_interface_.removeCollisionObjects({object_id});
      RCLCPP_INFO(this->get_logger(), "Requested removal of object '%s'", object_id.c_str());

      while ((this->now() - start) < timeout)
      {
        auto known_objects = planning_scene_interface_.getKnownObjectNames();
        if (std::find(known_objects.begin(), known_objects.end(), object_id) == known_objects.end())
        {
          success = true;
          break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Object '%s' successfully removed", object_id.c_str());
        response->success = true;
        response->message = "Object removed and verified";
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Timeout: object '%s' still present in scene", object_id.c_str());
        response->success = false;
        response->message = "Object removal timed out";
      }
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error in addObjectCallback: %s", e.what());
    response->success = false;
    response->message = e.what();
  }
}

void ObjectAttacherNode::attachObjectCallback(
  const std::shared_ptr<moveit_object_attacher::srv::AttachObj::Request> request,
  std::shared_ptr<moveit_object_attacher::srv::AttachObj::Response> response)
{
  const std::string& object_id = request->object_id;
  const std::string& frame_id = request->frame_id;
  const std::vector<std::string>& touch_links = request->touch_links;

  try
  {
    if (request->attach)
    {
      move_group_.attachObject(object_id, frame_id, touch_links);
      RCLCPP_INFO(this->get_logger(), "Attached object '%s' to '%s'", object_id.c_str(), frame_id.c_str());
      response->success = true;
      response->message = "Object attached";
    }
    else
    {
      move_group_.detachObject(object_id);
      RCLCPP_INFO(this->get_logger(), "Detached object '%s'", object_id.c_str());
      response->success = true;
      response->message = "Object detached";
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error in attachObjectCallback: %s", e.what());
    response->success = false;
    response->message = e.what();
  }
}

}  // namespace moveit_object_attacher

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<moveit_object_attacher::ObjectAttacherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
