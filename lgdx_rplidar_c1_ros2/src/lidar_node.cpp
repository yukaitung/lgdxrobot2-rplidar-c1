#include <format>
#include <numbers> 

#include "lgdx_rplidar_c1_ros2/lidar_node.hpp"
#include "lgdx_rplidar_c1_ros2/helper.hpp"

LidarNode::LidarNode() : Node("rplidar_c1_node")
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
  health_timer_ = this->create_wall_timer(std::chrono::milliseconds(kHealthRetryWaitMs), [this]() 
  {
    health_timer_->cancel();
    serial_port_->StartSerialThread();
    boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
  });
  health_timer_->cancel();
}

void LidarNode::Initalise()
{
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Starting lgdx_rplidar_c1_ros2");

  // Declare parameters
  auto serial_port_param = rcl_interfaces::msg::ParameterDescriptor{};
  serial_port_param.description = "Specifying usb port to connected lidar.";
  this->declare_parameter("serial_port", "/dev/ttyUSB0", serial_port_param);
  auto serial_baudrate_param = rcl_interfaces::msg::ParameterDescriptor{};
  serial_baudrate_param.description = "Specifying usb port baudrate to connected lidar.";
  this->declare_parameter("serial_baudrate", 460800, serial_baudrate_param);
  auto frame_id_param = rcl_interfaces::msg::ParameterDescriptor{};
  frame_id_param.description = "Specifying frame id of lidar.";
  this->declare_parameter("frame_id", "laser_frame", frame_id_param);
  auto inverted_param = rcl_interfaces::msg::ParameterDescriptor{};
  inverted_param.description = "Specifying whether or not to invert scan data.";
  this->declare_parameter("inverted", false, inverted_param);
  auto angle_compensate_param = rcl_interfaces::msg::ParameterDescriptor{};
  angle_compensate_param.description = "Specifying whether or not to enable angle_compensate of scan data.";
  this->declare_parameter("angle_compensate", false, angle_compensate_param);
  auto scan_mode_param = rcl_interfaces::msg::ParameterDescriptor{};
  scan_mode_param.description = "Specifying scan mode of lidar.";
  this->declare_parameter("scan_mode", "", scan_mode_param);

  // Set parameters
  angle_compensate_ = this->get_parameter("angle_compensate").as_bool();
  frame_id_ = this->get_parameter("frame_id").as_string();
  inverted_ = this->get_parameter("inverted").as_bool();

  // Publisher
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

  io_context_ = std::make_shared<boost::asio::io_context>();
  serial_port_ = std::make_shared<SerialPort>(shared_from_this(), io_context_);
  config_ = std::make_unique<Config>(serial_port_);
  scan_ = std::make_unique<Scan>(serial_port_);

  serial_port_->StartSerialThread();
  boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
}

boost::asio::awaitable<void> LidarNode::Main()
{
  // Self Check
  bool self_check = co_await SelfCheck();
  if (self_check == false)
  {
    co_await serial_port_->Reset();
    if (health_retry_count_ >= kMaxHealthRetry)
    {
      RCLCPP_FATAL(this->get_logger(), "The lidar is damaged, node will be terminated.");
      rclcpp::shutdown();
      co_return;
    }
    health_retry_count_++;
    health_timer_->reset();
    co_return;
  }

  // Set calculation
  int points_per_revolution = int(1000 * 1000 / current_scan_mode_.us_per_sample / scan_frequency_);
  angle_compensate_multiple = points_per_revolution / 360.0  + 1;
  if (angle_compensate_multiple < 1)
  {
    angle_compensate_multiple = 1.0;
  }
  
  // Scan
  co_await scan_->StartNormalScan();

  std::vector<LidarScanData> scans;
  scans.reserve(1024);
  rclcpp::Time start_time = this->now();
  while (rclcpp::ok())
  {
    auto [has_new_scan, some_scans] = co_await scan_->NormalScan();
    if (has_new_scan && scans.size() > 0)
    {
      rclcpp::Time end_time = this->now();
      float scan_time = (end_time - start_time).seconds();
      
      if (angle_compensate_)
      {
        const int angle_compensate_count = 360 * angle_compensate_multiple;
        int angle_compensate_offset = 0;
        std::vector<LidarScanData> angle_compensate_scans;
        angle_compensate_scans.reserve(scans.size());
        for (size_t i = 0, j = 0; i < scans.size(); i++)
        {
          if (scans[i].distance != 0.0f)
          {
            int angle_value = int(scans[i].angle * angle_compensate_multiple);
            if ((angle_value - angle_compensate_offset) < 0)
            {
              angle_compensate_offset = angle_compensate_count;
            }
            while (j < angle_compensate_multiple)
            {
              int angle_compensate_index = angle_value - angle_compensate_offset + j;
              if (angle_compensate_index >= angle_compensate_count)
              {
                angle_compensate_index = angle_compensate_count - 1;
              }
              angle_compensate_scans[angle_compensate_index] = scans[i];
            }
          }
        }

        float angle_max = Helper::DegToRad(360.0f);
        float angle_min = 0.0f;

        PublishScan(angle_compensate_scans, 0, angle_compensate_count, angle_max, angle_min, start_time, scan_time);
      }
      else
      {
        int start = 0, end = 0, i = 0;
        // Find the first valid node and last valid node
        while (scans[i++].distance == 0.0f && (long unsigned int)(i) < scans.size() - 1);
        start = i - 1;
        i = scans.size() - 1;
        while (scans[i--].distance != 0.0f && i > 0);
        end = i + 1;

        float angle_max = scans[end].angle;
        float angle_min = scans[start].angle;

        PublishScan(scans, start, end - start + 1, angle_max, angle_min, start_time, scan_time);
      }

      scans.clear();
      start_time = this->now();
    }
    scans.insert(scans.end(), some_scans.begin(), some_scans.end());
  }
}

boost::asio::awaitable<bool> LidarNode::SelfCheck()
{
  RCLCPP_INFO(this->get_logger(), "Starting self check");

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
      .us_per_sample = us_per_sample,
      .sample_rate =  1.0f / (float)us_per_sample * 1000.0f,
      .max_distance = max_distance,
      .answer_type = ans_type
    };
    scan_modes[name] = scan_mode;
  }
  for(auto [key, scan_mode] : scan_modes)
  {
    // TODO: Change mode
    if (scan_mode.mode == 1)
    {
      current_scan_mode_ = scan_mode;
    }
    RCLCPP_INFO(this->get_logger(), "Index: %d, Scan Mode: %s, Sample Rate: %.0f kHz, Max Distance: %d m, Answer Type: 0x%x",
      scan_mode.mode, key.c_str(), scan_mode.sample_rate, scan_mode.max_distance, scan_mode.answer_type);
  }

  // Get Health
  LidarHealth health = co_await config_->GetHealth();
  switch(health.status)
  {
    case 0:
      RCLCPP_INFO(this->get_logger(), "Health status: OK");
      co_return true;
      break;
    case 1:
      RCLCPP_WARN(this->get_logger(), "Health status: Warning, Error Code: %d", health.error_code);
      co_return true;
      break;
    case 2:
      RCLCPP_ERROR(this->get_logger(), "Health status: Error, Error Code: %d, will be restarted", health.error_code);
      co_return false;
  }
}

void LidarNode::PublishScan(const std::vector<LidarScanData> &scans, 
  size_t valid_scan_start, size_t valid_scans_count,
  const float angle_max, const float angle_min,
  const rclcpp::Time &start_time, const float scan_time)
{
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp = start_time;
  scan_msg.header.frame_id = frame_id_;

  bool reversed = (angle_max > angle_min);
  if (reversed)
  {
    scan_msg.angle_min = std::numbers::pi - angle_max;
    scan_msg.angle_max = std::numbers::pi - angle_min;
  }
  else
  {
    scan_msg.angle_min = std::numbers::pi - angle_min;
    scan_msg.angle_max = std::numbers::pi - angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / float(valid_scans_count - 1);
  scan_msg.scan_time = scan_time;
  scan_msg.range_min = kScanMinDistance;
  scan_msg.range_max = current_scan_mode_.max_distance;

  scan_msg.intensities.reserve(valid_scans_count);
  scan_msg.ranges.reserve(valid_scans_count);
  for (size_t i = valid_scan_start; i < valid_scans_count; i++)
  {
    if (scans[i].distance == 0.0f)
    {
      scan_msg.ranges.push_back(std::numeric_limits<float>::infinity());
    }
    else
    {
      scan_msg.ranges.push_back(scans[i].distance);
    }
    scan_msg.intensities.push_back(scans[i].quality >> 2);
  }
  bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
  if (reverse_data)
  {
    std::reverse(scan_msg.ranges.begin(), scan_msg.ranges.end());
    std::reverse(scan_msg.intensities.begin(), scan_msg.intensities.end());
  }
  scan_pub_->publish(scan_msg);
}