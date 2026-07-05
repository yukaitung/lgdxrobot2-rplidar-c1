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
    Helper::PrintHex(data);
    buffer_.insert(buffer_.end(), data.begin(), data.end());
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + kDescriptorSize);
}

boost::asio::awaitable<std::vector<LidarScanData>> ExpressScan::GetData()
{

  std::vector<LidarScanData> scans;

  co_return scans;
}