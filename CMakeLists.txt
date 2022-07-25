cmake_minimum_required(VERSION 3.8)
project(boeing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(alglib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(gcode src/arm_planner.cpp src/gcode_reader.cpp src/serial_manager.cpp src/traj_perc.cpp src/readGCodeV3.cpp)

ament_target_dependencies(gcode 
  rclcpp 
  std_msgs 
  geometry_msgs 
  moveit_msgs
  Eigen3 
  alglib 
  Boost 
  moveit_ros_planning_interface 
)


target_link_libraries(gcode alglib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Arm2Proc.msg"
  "msg/PointList.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

target_include_directories(gcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gcode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  gcode
  DESTINATION lib/${PROJECT_NAME})

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
