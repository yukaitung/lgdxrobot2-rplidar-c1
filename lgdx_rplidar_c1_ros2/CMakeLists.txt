cmake_minimum_required(VERSION 3.20)
project(lgdx_rplidar_c1_ros2)

set(EXECUTABLE_NAME rplidar_c1_node)

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
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(${EXECUTABLE_NAME} 
  src/main.cpp
  src/config.cpp
  src/helper.cpp
  src/lidar_node.cpp
  src/scan.cpp
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
  std_msgs::std_msgs
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${EXECUTABLE_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
