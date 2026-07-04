#include "lgdx_rplidar_c1_ros2/lidar_node.hpp"

#include <format>

LidarNode::LidarNode() : Node("lidar_node")
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void LidarNode::Initalise()
{
  timer_->cancel();

  io_context_ = std::make_shared<boost::asio::io_context>();
  serial_port_ = std::make_shared<SerialPort>(shared_from_this(), io_context_);
  config_ = std::make_unique<Config>(serial_port_);

  serial_port_->StartSerialThread();
  boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
}

boost::asio::awaitable<void> LidarNode::Main()
{
  LidarInfo info = co_await config_->GetInfo();
  uint8_t major_model = info.model >> 4;
  uint8_t minor_model = info.model & 0x0F;
  std::string serial_number;
  for(auto i = 0; i < 16; i++)
  {
    serial_number += std::format("{:02X}", info.serial_number[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Major Model: %d, Minor Model: %d", major_model, minor_model);
  RCLCPP_INFO(this->get_logger(), "Firmware Version: %d.%d", info.firmware_major, info.firmware_minor);
  RCLCPP_INFO(this->get_logger(), "Hardware Version: %d", info.hardware);
  RCLCPP_INFO(this->get_logger(), "Serial Number: %s", serial_number.c_str());
}