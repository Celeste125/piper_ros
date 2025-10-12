cmake_minimum_required(VERSION 3.16)
project(piper_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)


install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  PROGRAMS scripts/keyboard_controller.py
  PROGRAMS scripts/moveilt_keyboard_control.py
  PROGRAMS scripts/rviz_keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar archivos de lanzamiento - CORREGIDO
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
  # PATTERN "*.launch.py"  # Opcional: ser más específico
)

# Crear librería de cinemática primero
add_library(piper_kinematics_lib
  src/piper_forward_kinematics.cpp
  src/piper_jacobian_ik.cpp
  src/piper_analytical_ik.cpp
)
target_include_directories(piper_kinematics_lib PUBLIC
  include
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(piper_kinematics_lib
  rclcpp
  Eigen3
)

# Luego el ejecutable cartesian_controller
add_executable(cartesian_controller src/cartesian_controller.cpp)
target_include_directories(cartesian_controller PRIVATE
  include
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(cartesian_controller 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Linkear contra nuestra librería
target_link_libraries(cartesian_controller piper_kinematics_lib)

# Instalar ambos
install(TARGETS piper_kinematics_lib cartesian_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()