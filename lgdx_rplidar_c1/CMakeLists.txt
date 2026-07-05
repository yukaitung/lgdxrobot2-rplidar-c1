cmake_minimum_required(VERSION 3.20)
project(lgdx_rplidar_c1)

set(EXECUTABLE_NAME rplidar_c1_component)
set(EXPORT_EXECUTABLE_NAME export_rplidar_c1_component)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
if (POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(${EXECUTABLE_NAME} 
  src/config.cpp
  src/helper.cpp
  src/lidar_node.cpp
  src/scan/express_scan.cpp
  src/scan/scan.cpp
  src/serial_port.cpp
)
target_link_libraries(${EXECUTABLE_NAME}
  "${cpp_typesupport_target}"
  Boost::headers
)
target_link_libraries(${EXECUTABLE_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs
)

rclcpp_components_register_node(${EXECUTABLE_NAME}
  PLUGIN "LgdxRobot2::LidarNode"
  EXECUTABLE rplidar_c1_node
)
ament_export_targets(${EXPORT_EXECUTABLE_NAME})
install(TARGETS ${EXECUTABLE_NAME}
  EXPORT ${EXPORT_EXECUTABLE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
