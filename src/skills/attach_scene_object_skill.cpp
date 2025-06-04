#include <moveit_object_attacher/skills/attach_scene_object_skill.hpp>

MoveitAttachObjectSkill::MoveitAttachObjectSkill(const std::string& name,
                                                 const BT::NodeConfig& conf,
                                                 const BT::RosNodeParams& params)
  : BT::RosServiceNode<moveit_object_attacher::srv::AttachObj>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool MoveitAttachObjectSkill::setRequest(Request::SharedPtr& goal)
{
  std::string w;

  // attach flag
  if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/attach", goal->attach, w))
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w.c_str());
    return false;
  }

  // object_id
  if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/object_id", goal->object_id, w))
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
    return false;
  }

  if(goal->attach)
  {
    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/frame_id", goal->frame_id, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }

    if (!bt_executer::utils::get_param(node_.lock().get(), ns_, "/touch_links", goal->touch_links, w))
    {
      RCLCPP_ERROR_STREAM(node_.lock()->get_logger(),w);
      return false;
    }
  }

  return true;
}

BT::NodeStatus MoveitAttachObjectSkill::onResponseReceived(const Response::SharedPtr& response)
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

BT::NodeStatus MoveitAttachObjectSkill::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class MoveitAttachObjectSkill will self register with name  "MoveitAttachObjectSkill".
CreateRosNodePlugin(MoveitAttachObjectSkill, "MoveitAttachObjectSkill");
