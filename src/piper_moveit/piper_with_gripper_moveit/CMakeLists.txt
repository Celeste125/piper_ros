cmake_minimum_required(VERSION 3.22)
project(piper_with_gripper_moveit)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(piper_pose_goal src/piper_pose_goal.cpp)

ament_target_dependencies(piper_pose_goal
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)

install(TARGETS piper_pose_goal
  DESTINATION lib/${PROJECT_NAME})

  
ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
