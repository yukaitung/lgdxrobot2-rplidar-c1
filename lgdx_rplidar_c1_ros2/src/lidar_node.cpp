#include "lgdx_rplidar_c1_ros2/lidar_node.hpp"

#include <format>

LidarNode::LidarNode() : Node("rplidar_c1_node")
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void LidarNode::Initalise()
{
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Starting lgdx_rplidar_c1_ros2");

  io_context_ = std::make_shared<boost::asio::io_context>();
  serial_port_ = std::make_shared<SerialPort>(shared_from_this(), io_context_);
  config_ = std::make_unique<Config>(serial_port_);

  serial_port_->StartSerialThread();
  boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
}

boost::asio::awaitable<void> LidarNode::Main()
{
  co_await LidarInitalise();
}

boost::asio::awaitable<void> LidarNode::LidarInitalise()
{
  // Get Hardware Info
  LidarInfo info = co_await config_->GetInfo();
  uint8_t major_model = info.model >> 4;
  uint8_t sub_model = info.model & 0x0F;
  std::string serial_number;
  for(auto i = 0; i < 16; i++)
  {
    serial_number += std::format("{:02X}", info.serial_number[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Major Model: %d, Sub Model: %d, Firmware Version: %d.%d, Hardware Version: %d, Serial Number: %s"
    , major_model, sub_model, info.firmware_major, info.firmware_minor, info.hardware, serial_number.c_str());

  // Get Scan Mode
  uint16_t scan_mode_count = co_await config_->GetScanModeCount();
  uint16_t typical = co_await config_->GetScanModeTypical();
  RCLCPP_INFO(this->get_logger(), "Scan Mode Count: %d, Scan Mode Typical: %d", scan_mode_count, typical);
  std::map<std::string, LidarScanMode> scan_modes;
  for(auto i = 0; i < scan_mode_count; i++)
  {
    uint32_t us_per_sample = co_await config_->GetScanModeUsPerSample(i);
    uint32_t max_distance = co_await config_->GetScanModeMaxDistance(i);
    uint8_t ans_type = co_await config_->GetScanModeAnsType(i);
    std::string name = co_await config_->GetScanModeName(i);
    LidarScanMode scan_mode{
      .mode = i + 1,
      .sample_rate =  1.0f / (float)us_per_sample * 1000.0f,
      .max_distance = max_distance,
      .answer_type = ans_type
    };
    scan_modes[name] = scan_mode;
  }
  for(auto [key, scan_mode] : scan_modes)
  {
    RCLCPP_INFO(this->get_logger(), "Index: %d, Scan Mode: %s, Sample Rate: %.0f kHz, Max Distance: %d m, Answer Type: 0x%x",
      scan_mode.mode, key.c_str(), scan_mode.sample_rate, scan_mode.max_distance, scan_mode.answer_type);
  }

  // Get Health
  LidarHealth health = co_await config_->GetHealth();
  RCLCPP_INFO(this->get_logger(), "Health status: %d, Error Code: %d", health.status, health.error_code);
}