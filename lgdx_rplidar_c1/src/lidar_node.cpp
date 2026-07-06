#include <format>
#include <numbers> 
#include <rclcpp_components/register_node_macro.hpp>

#include "lgdx_rplidar_c1/lidar_node.hpp"
#include "lgdx_rplidar_c1/helper.hpp"
#include "lgdx_rplidar_c1/exceptions/get_config_exception.hpp"
#include "lgdx_rplidar_c1/exceptions/serial_port_exception.hpp"
#include "lgdx_rplidar_c1/scan/scan.hpp"
#include "lgdx_rplidar_c1/scan/express_scan.hpp"

namespace LgdxRobot2 
{

LidarNode::LidarNode(const rclcpp::NodeOptions &options) : Node("rplidar_c1_node", options)
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
  health_timer_ = this->create_wall_timer(std::chrono::milliseconds(kHealthRetryWaitMs), [this]() 
  {
    health_timer_->cancel();
    boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
    serial_port_->StartSerialThread();
  });
  health_timer_->cancel();
  retry_timer_ = this->create_wall_timer(std::chrono::seconds(kRetryWaitSecond), [this]() 
  {
    retry_timer_->cancel();
    ConnectSerialPort();
  });
  retry_timer_->cancel();

  rclcpp::on_shutdown([this]()
  {
    if (serial_port_)
    {
      serial_port_->Shutdown();
    }
  });
}

void LidarNode::Initalise()
{
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Starting lgdx_rplidar_c1");

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
  this->declare_parameter("scan_mode", "Standard", scan_mode_param);

  // Set parameters
  angle_compensate_ = this->get_parameter("angle_compensate").as_bool();
  frame_id_ = this->get_parameter("frame_id").as_string();
  inverted_ = this->get_parameter("inverted").as_bool();
  scam_mode_ = this->get_parameter("scan_mode").as_string();

  // Publisher
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

  io_context_ = std::make_shared<boost::asio::io_context>();
  serial_port_ = std::make_shared<SerialPort>(shared_from_this(), io_context_);
  config_ = std::make_unique<Config>(serial_port_);

  ConnectSerialPort();
}

void LidarNode::ConnectSerialPort()
{
  try
  {
    serial_port_->Connect();
    boost::asio::co_spawn(*io_context_, LidarNode::Main(), boost::asio::detached);
    serial_port_->StartSerialThread();
  }
  catch(const SerialPortException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Serial port exception, reconnecting in %d seconds: %s", kRetryWaitSecond, e.what());
    retry_timer_->reset();
    return;
  }
}

boost::asio::awaitable<void> LidarNode::Main()
{
  RCLCPP_INFO(this->get_logger(), "Starting main");
  try
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
    angle_compensate_multiple = points_per_revolution / 360.0 + 1;
    if (angle_compensate_multiple < 1)
    {
      angle_compensate_multiple = 1.0;
    }
    
    // Scan
    co_await scan_->Start();

    std::vector<LidarScanData> scans;
    scans.reserve(1024);
    rclcpp::Time start_time = this->now();
    float last_angle = -1.0f;
    while (rclcpp::ok())
    {
      auto some_scans = co_await scan_->GetData();
      if (!rclcpp::ok())
      {
        co_return;
      }

      // Check if new scan data is available
      bool has_new_scan = false;
      size_t new_scan_index = 0;
      for (size_t i = 0; i < some_scans.size(); i++)
      {
        if (some_scans[i].angle < last_angle)
        {
          has_new_scan = true;
          new_scan_index = i;
        }
        last_angle = some_scans[i].angle;
      }

      // Process the scan data for one revolution
      if (has_new_scan && scans.size() > 0)
      {
        if (new_scan_index > 0)
        {
          // The frame contains the new scan data, append the data before the new scan data
          scans.insert(scans.end(), some_scans.begin(), some_scans.begin() + new_scan_index - 1);
        }

        rclcpp::Time end_time = this->now();
        float scan_time = (end_time - start_time).seconds();
        
        if (angle_compensate_)
        {
          const int angle_compensate_count = 360 * angle_compensate_multiple;
          int angle_compensate_offset = 0;
          std::vector<LidarScanData> angle_compensate_scans(angle_compensate_count, LidarScanData{});
          for (size_t i = 0; i < scans.size(); i++)
          {
            if (scans[i].distance != 0.0f)
            {
              int angle_value = int(scans[i].angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0)
              {
                angle_compensate_offset = angle_value;
              }
              for (size_t j = 0; j < angle_compensate_multiple; j++)
              {
                int angle_compensate_index = angle_value - angle_compensate_offset + j;
                if (angle_compensate_index >= angle_compensate_count)
                {
                  angle_compensate_index = angle_compensate_count - 1;
                }
                if (angle_compensate_index >= 0 && angle_compensate_index < angle_compensate_count) // Must > 0
                {
                  angle_compensate_scans[angle_compensate_index] = scans[i];
                }
              }
            }
          }

          float angle_max = Helper::DegToRad(360.0f);
          float angle_min = 0.0f;

          PublishScan(angle_compensate_scans, 0, angle_compensate_scans.size(), angle_max, angle_min, start_time, scan_time);
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

          float angle_max = Helper::DegToRad(scans[end].angle);
          float angle_min = Helper::DegToRad(scans[start].angle);

          PublishScan(scans, start, end - start + 1, angle_max, angle_min, start_time, scan_time);
        }

        scans.clear();
        start_time = this->now();
        scans.insert(scans.end(), some_scans.begin() + new_scan_index, some_scans.end());
      }
      else
      {
        // Not full revolution, append the scan data
        scans.insert(scans.end(), some_scans.begin(), some_scans.end());
      }
    }
  }
  catch(const GetConfigException& e)
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Get config exception, reconnecting in %d seconds: %s", kRetryWaitSecond, e.what());
      retry_timer_->reset();
    }
    co_return;
  }
  catch(const SerialPortException& e)
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Serial port exception, reconnecting in %d seconds: %s", kRetryWaitSecond, e.what());
      retry_timer_->reset();
    }
    co_return;
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
    if (name[name.length() - 1] == '\0')
    {
      name.pop_back();
    }
    LidarScanMode scan_mode{
      .mode = i + 1,
      .name = name,
      .us_per_sample = us_per_sample,
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

  // set scan mode if specified
  if (scam_mode_ != "Standard")
  {
    if (auto search = scan_modes.find(scam_mode_); search != scan_modes.end())
    {
      current_scan_mode_ = search->second;
      scan_ = std::make_unique<ExpressScan>(serial_port_, current_scan_mode_.answer_type);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "The %s scan mode is not supported.", scam_mode_.c_str());
      current_scan_mode_ = scan_modes["Standard"];
      scan_ = std::make_unique<Scan>(serial_port_);
    }
  }
  else
  {
    current_scan_mode_ = scan_modes["Standard"];
    scan_ = std::make_unique<Scan>(serial_port_);
  }
  RCLCPP_INFO(this->get_logger(), "Current Scan Mode: %s", current_scan_mode_.name.c_str());


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
    scan_msg.angle_min = std::numbers::pi_v<float> - angle_max;
    scan_msg.angle_max = std::numbers::pi_v<float> - angle_min;
  }
  else
  {
    scan_msg.angle_min = std::numbers::pi_v<float> - angle_min;
    scan_msg.angle_max = std::numbers::pi_v<float> - angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / float(valid_scans_count - 1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / float(valid_scans_count - 1);
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

}

RCLCPP_COMPONENTS_REGISTER_NODE(LgdxRobot2::LidarNode)
