#include "lgdx_rplidar_c1_ros2/scan/express_scan.hpp"
#include "lgdx_rplidar_c1_ros2/helper.hpp"

ExpressScan::ExpressScan(std::shared_ptr<SerialPort> serial_port, uint8_t mode) :
  mode_(mode),
  serial_port_(serial_port),
  buffer_(kBufferSize)
{}

boost::asio::awaitable<void> ExpressScan::Start()
{
  // Send the command
  std::vector<uint8_t> command = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
  command[3] = mode_;
  co_await serial_port_->Write(command);

  while (rclcpp::ok() && buffer_.size() < kDescriptorSize)
  {
    auto data = co_await serial_port_->Read();
    buffer_.insert(buffer_.end(), data.begin(), data.end());
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + kDescriptorSize);
}

boost::asio::awaitable<std::vector<LidarScanData>> ExpressScan::GetData()
{
  const int kExpectedSize = 40;
  const int kDistantDataOffser = 4;

  // Wait till buffer contains enough bytes
  while (buffer_.size() < (kScanDataSize + 4))
  {
    auto data = co_await serial_port_->Read();
    //Helper::PrintHex(data);
    buffer_.insert(buffer_.end(), data.begin(), data.end());
  }
  
  std::vector<LidarScanData> scans;
  scans.reserve(kExpectedSize);

  float start_angle = uint16_t((buffer_[3] & 0x3F) << 8 | buffer_[2]) / 64.0f;
  float next_start_angle = uint16_t((buffer_[kScanDataSize + 3] & 0x3F) << 8 | buffer_[kScanDataSize + 2]) / 64.0f;

  for (int i = 0; i < kExpectedSize; i++)
  {
    float angle = start_angle + (AngleDiff(start_angle, next_start_angle) / kExpectedSize) * i;
    if (angle >= 360.0f)
    {
      angle -= 360.0f;
    }

    uint8_t upper_distance = buffer_[kDistantDataOffser + i * 2 + 1];
    uint8_t lower_distance = buffer_[kDistantDataOffser + i * 2];

    LidarScanData scan{
      .quality = 0,
      .angle = angle,
      .distance = (uint16_t(upper_distance << 8 | lower_distance) / 4.0f / 1000.0f)
    };
    scans.push_back(scan);
  }
  
  buffer_.erase(buffer_.begin(), buffer_.begin() + kScanDataSize);

  co_return scans;
}

float ExpressScan::AngleDiff(float this_angle, float next_angle)
{
  if (this_angle <= next_angle)
  {
    return next_angle - this_angle;
  }
  else
  {
    return (360.0f + next_angle) - this_angle;
  }
}