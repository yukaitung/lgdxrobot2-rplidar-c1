#include "lgdx_rplidar_c1_ros2/config.hpp"

#include <bit>
#include <cstring>

Config::Config(std::shared_ptr<SerialPort> serial_port) :
  serial_port_(serial_port)
{}

uint8_t Config::GetCheckSum(const std::vector<uint8_t> &data)
{
  uint8_t checksum = 0;
  for(auto d : data)
  {
    checksum ^= d;
  }
  return checksum;
}

boost::asio::awaitable<LidarInfo> Config::GetInfo()
{
  std::vector<uint8_t> command = {0xA5, 0x50};
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  LidarInfo info;
  std::memcpy(&info, data.data(), sizeof(LidarInfo));
  co_return info;
}

boost::asio::awaitable<LidarHealth> Config::GetHealth()
{
  std::vector<uint8_t> command = {0xA5, 0x52};
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  LidarHealth health;
  std::memcpy(&health, data.data(), sizeof(LidarHealth));
  co_return health;
}

boost::asio::awaitable<LidarSampleRate> Config::GetSampleRate()
{
  std::vector<uint8_t> command = {0xA5, 0x59};
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  LidarSampleRate sample_rate;
  std::memcpy(&sample_rate, data.data(), sizeof(LidarSampleRate));
  co_return sample_rate;
}

boost::asio::awaitable<uint16_t> Config::GetScanModeCount()
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x70};
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  uint16_t count;
  std::memcpy(&count, data.data(), sizeof(uint16_t));
  co_return count;
}

boost::asio::awaitable<uint32_t> Config::GetScanModeUsPerSample(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x71, 0x00, 0x00};
  command[3] = index >> 8;
  command[4] = index & 0xFF;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  uint32_t us_per_sample;
  std::memcpy(&us_per_sample, data.data(), sizeof(uint32_t));
  co_return us_per_sample;
}

boost::asio::awaitable<uint32_t> Config::GetScanModeMaxDistance(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x74, 0x00, 0x00};
  command[3] = index >> 8;
  command[4] = index & 0xFF;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  uint32_t max_distance;
  std::memcpy(&max_distance, data.data(), sizeof(uint32_t));
  co_return max_distance;
}

boost::asio::awaitable<uint8_t> Config::GetScanModeAnsType(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x75, 0x00, 0x00};
  command[3] = index >> 8;
  command[4] = index & 0xFF;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  uint8_t ans_type;
  std::memcpy(&ans_type, data.data(), sizeof(uint8_t));
  co_return ans_type;
}

boost::asio::awaitable<uint16_t> Config::GetScanModeTypical()
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x7C};
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  uint16_t typical;
  std::memcpy(&typical, data.data(), sizeof(uint16_t));
  co_return typical;
}

boost::asio::awaitable<std::string> Config::GetScanModeName(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x7F, 0x00, 0x00};
  command[3] = index >> 8;
  command[4] = index & 0xFF;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  std::string name(data.begin(), data.end());
  co_return name;
}