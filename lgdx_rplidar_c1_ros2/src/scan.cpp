#include "lgdx_rplidar_c1_ros2/scan.hpp"
#include "lgdx_rplidar_c1_ros2/helper.hpp"

Scan::Scan(std::shared_ptr<SerialPort> serial_port) :
  serial_port_(serial_port),
  buffer_(kBufferSize)
{}

boost::asio::awaitable<void> Scan::StartNormalScan()
{
  // Send the command
  std::vector<uint8_t> command = {0xA5, 0x20};
  co_await serial_port_->Write(command);

  while (rclcpp::ok() && buffer_.size() < kDescriptorSize)
  {
    auto data = co_await serial_port_->Read();
    buffer_.insert(buffer_.end(), data.begin(), data.end());
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + kDescriptorSize);
}

boost::asio::awaitable<std::vector<LidarScanData>> Scan::NormalScan()
{
  auto data = co_await serial_port_->Read();
  buffer_.insert(buffer_.end(), data.begin(), data.end());

  std::vector<LidarScanData> scans;
  scans.reserve(40);
  while (rclcpp::ok() && buffer_.size() >= kScanDataSize)
  {
    std::vector<uint8_t> scan_data(buffer_.begin(), buffer_.begin() + kScanDataSize);
    LidarScanData scan{
      .quality = uint8_t(scan_data[0] >> 2),
      .angle = uint16_t(scan_data[2] << 7 | scan_data[1] >> 1) / 64.0f,
      .distance = (uint16_t(scan_data[4] << 8 | scan_data[3]) / 4.0f / 1000.0f)
    };
    scans.push_back(scan);
    buffer_.erase(buffer_.begin(), buffer_.begin() + kScanDataSize);
  }

  co_return scans;
}