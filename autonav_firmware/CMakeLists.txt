cmake_minimum_required(VERSION 3.8)
project(autonav_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

add_library(${PROJECT_NAME} SHARED
  src/autonav_interface.cpp
  src/motor_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs
)

target_link_libraries(${PROJECT_NAME} ${MODBUS_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${MODBUS_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

include_directories(
  include
)

# ESP32 serial bridge node: device identity, LED patterns, physical e-stop
# feedback. Independent of the ros2_control plugin above -- plain executable.
add_executable(esp32_bridge_node
  src/esp32_bridge_node.cpp
  src/esp32_serial_transport.cpp
  src/esp32_frame_codec.cpp
)
target_link_libraries(esp32_bridge_node
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  autonav_msgs::autonav_msgs
)

install(
  TARGETS esp32_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface autonav_interface.xml)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
