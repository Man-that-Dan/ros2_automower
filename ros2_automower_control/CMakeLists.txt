cmake_minimum_required(VERSION 3.5)
project(ros2_automower_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)



add_library(ros2_automower_control SHARED src/motor_interface.cpp src/wheel.cpp src/arduino_comms.cpp)

target_include_directories(
  ros2_automower_control PUBLIC
  include
)



ament_target_dependencies(
  ros2_automower_control PUBLIC
  hardware_interface
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

target_link_libraries(ros2_automower_control PUBLIC serial)

pluginlib_export_plugin_description_file(hardware_interface ros2_automower_control.xml)

add_executable(blade_controller src/blade_controller.cpp src/arduino_comms.cpp src/wheel.cpp)
target_include_directories(blade_controller PRIVATE include)
ament_target_dependencies(blade_controller PUBLIC
  serial
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BladeStatus.msg"
  "srv/BladeCommand.srv"
 )
#target_link_libraries(ros2_automower_control PUBLIC serial)


install(
  DIRECTORY config launch ros2_control
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ros2_automower_control
  EXPORT export_ros2_automower_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS blade_controller
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_ros2_automower_control HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  )
ament_package()
