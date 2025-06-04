cmake_minimum_required(VERSION 3.10)
project(moveit_object_attacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast -O3)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cnr_param REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(SKILL_LIBRARY_DEPS
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    std_srvs
    geometry_msgs
    shape_msgs
    behaviortree_ros2
    btcpp_ros2_interfaces
    )

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddObj.srv"
  "srv/AttachObj.srv"
  DEPENDENCIES moveit_msgs
)

# Executable
add_executable(object_attacher_node src/object_attacher_node.cpp)
target_include_directories(object_attacher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(object_attacher_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  std_srvs
  geometry_msgs
  shape_msgs
)

#Skill library
add_library(moveit_add_object_plugin SHARED
    src/skills/add_scene_object_skill.cpp
    )
target_link_libraries(moveit_add_object_plugin cnr_param::cnr_param)
target_compile_definitions(moveit_add_object_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(moveit_add_object_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(moveit_add_object_plugin ${SKILL_LIBRARY_DEPS})

add_library(moveit_attach_object_plugin SHARED
    src/skills/attach_scene_object_skill.cpp
    )
target_link_libraries(moveit_attach_object_plugin cnr_param::cnr_param)
target_compile_definitions(moveit_attach_object_plugin PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(moveit_attach_object_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(moveit_attach_object_plugin ${SKILL_LIBRARY_DEPS})

# Include generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(object_attacher_node ${cpp_typesupport_target})
target_link_libraries(moveit_add_object_plugin ${cpp_typesupport_target})
target_link_libraries(moveit_attach_object_plugin ${cpp_typesupport_target})

# Install targets
install(TARGETS
  object_attacher_node
  moveit_add_object_plugin
  moveit_attach_object_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export
ament_export_dependencies(rosidl_default_runtime)
ament_package()
