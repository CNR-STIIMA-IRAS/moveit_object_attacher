#pragma once

#include "moveit_object_attacher/skills/utils.hpp"
#include "moveit_object_attacher/srv/add_obj.hpp"

class MoveitAddObjectSkill: public BT::RosServiceNode<moveit_object_attacher::srv::AddObj>
{
public:
  MoveitAddObjectSkill(const std::string& name,
                       const BT::NodeConfig& conf,
                       const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
          {
            BT::InputPort<std::string>("param_ns")
          }
          );
  }

  bool setRequest(Request::SharedPtr& goal) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string ns_;
};
