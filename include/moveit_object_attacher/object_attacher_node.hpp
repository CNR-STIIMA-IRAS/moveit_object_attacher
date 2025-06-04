#ifndef MOVEIT_OBJECT_ATTACHER__OBJECT_ATTACHER_NODE_HPP_
#define MOVEIT_OBJECT_ATTACHER__OBJECT_ATTACHER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include "moveit_object_attacher/srv/add_obj.hpp"
#include "moveit_object_attacher/srv/attach_obj.hpp"

namespace moveit_object_attacher
{

class ObjectAttacherNode : public rclcpp::Node
{
public:
  ObjectAttacherNode();

private:
  rclcpp::Service<moveit_object_attacher::srv::AddObj>::SharedPtr add_obj_srv_;
  rclcpp::Service<moveit_object_attacher::srv::AttachObj>::SharedPtr attach_obj_srv_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  void addObjectCallback(
    const std::shared_ptr<moveit_object_attacher::srv::AddObj::Request> request,
    std::shared_ptr<moveit_object_attacher::srv::AddObj::Response> response);

  void attachObjectCallback(
    const std::shared_ptr<moveit_object_attacher::srv::AttachObj::Request> request,
    std::shared_ptr<moveit_object_attacher::srv::AttachObj::Response> response);
};

}  // namespace moveit_object_attacher

#endif  // MOVEIT_OBJECT_ATTACHER__OBJECT_ATTACHER_NODE_HPP_
