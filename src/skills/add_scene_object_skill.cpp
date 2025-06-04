#include <moveit_object_attacher/skills/add_scene_object_skill.hpp>

MoveitAddObjectSkill::MoveitAddObjectSkill(const std::string& name,
                                           const BT::NodeConfig& conf,
                                           const BT::RosNodeParams& params)
  : BT::RosServiceNode<moveit_object_attacher::srv::AddObj>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool MoveitAddObjectSkill::setRequest(Request::SharedPtr& goal)
{
  std::string w;
  moveit_msgs::msg::CollisionObject object;

  bool add;
  if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/add", add, w))
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
    return false;
  }

  if(add)
  {
    object.operation = object.ADD;

    // Header: frame_id
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/frame_id", object.header.frame_id, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    object.header.stamp = node_.lock()->get_clock()->now();

    // ID
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/object_id", object.id, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    // Primitive shape type
    std::string primitive_type_str;
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/primitive_type", primitive_type_str, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    shape_msgs::msg::SolidPrimitive primitive;

    if (primitive_type_str == "box")
    {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/x", primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/y", primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/z", primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
    }
    else if (primitive_type_str == "sphere")
    {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions.resize(1);

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/radius", primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
    }
    else if (primitive_type_str == "cylinder")
    {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/height", primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/radius", primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
    }
    else if (primitive_type_str == "cone")
    {
      primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
      primitive.dimensions.resize(2);

      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/height", primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
      if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/dimensions/radius", primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS], w))
      {
        RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_.lock()->get_logger(), "Unsupported primitive type: %s", primitive_type_str.c_str());
      return false;
    }

    object.primitives.push_back(primitive);

    // Pose
    geometry_msgs::msg::Pose pose;
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/position/x", pose.position.x, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/position/y", pose.position.y, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/position/z", pose.position.z, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/orientation/x", pose.orientation.x, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/orientation/y", pose.orientation.y, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/orientation/z", pose.orientation.z, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/pose/orientation/w", pose.orientation.w, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    object.primitive_poses.push_back(pose);
  }
  else
  {
    object.operation = object.REMOVE;

    // ID
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/object_id", object.id, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }
  }

  goal->object = object;
  goal->add = add;

  return true;
}

BT::NodeStatus MoveitAddObjectSkill::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResponseReceived. Done = %s", name().c_str(),
              response->success ? "true" : "false");
  if (response->success)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveitAddObjectSkill::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class MoveitAddObjectSkill will self register with name  "MoveitAddObjectSkill".
CreateRosNodePlugin(MoveitAddObjectSkill, "MoveitAddObjectSkill");
