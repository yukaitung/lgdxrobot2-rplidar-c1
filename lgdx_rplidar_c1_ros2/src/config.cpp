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
  if (data.size() != 27 && data.at(7) != 0x50)
  {
    throw std::runtime_error("GetInfo: Unexpected data size");
  }
  LidarInfo info;
  std::memcpy(&info, data.data(), sizeof(LidarInfo));
  co_return info;
}

boost::asio::awaitable<LidarHealth> Config::GetHealth()
{
  std::vector<uint8_t> command = {0xA5, 0x52};
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 10 && data.at(7) != 0x52)
  {
    throw std::runtime_error("GetHealth: Unexpected data size");
  }
  LidarHealth health;
  std::memcpy(&health, data.data(), sizeof(LidarHealth));
  health.error_code = data.at(9) << 8 | data.at(8);
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
  std::vector<uint8_t> command = {0xA5, 0x84, 0x04, 0x70, 0x00, 0x00, 0x00};
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 13 && data.at(7) != 0x70)
  {
    throw std::runtime_error("GetScanModeCount: Unexpected data size");
  }
  uint16_t count = data.at(12) << 8 | data.at(11);
  co_return count;
}

boost::asio::awaitable<uint32_t> Config::GetScanModeUsPerSample(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x06, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00};
  command[7] = index & 0xFF;
  command[8] = index >> 8;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 15 && data.at(7) != 0x71)
  {
    throw std::runtime_error("GetScanModeUsPerSample: Unexpected data size");
  }
  uint32_t us_per_sample = data.at(13) << 24 | data.at(14) << 16 | data.at(11) << 8 | data.at(12);
  co_return us_per_sample;
}

boost::asio::awaitable<uint32_t> Config::GetScanModeMaxDistance(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x06, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00};
  command[7] = index & 0xFF;
  command[8] = index >> 8;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 15 && data.at(7) != 0x74)
  {
    throw std::runtime_error("GetScanModeMaxDistance: Unexpected data size");
  }
  uint32_t max_distance = data.at(13) << 24 | data.at(14) << 16 | data.at(11) << 8 | data.at(12);
  co_return max_distance;
}

boost::asio::awaitable<uint8_t> Config::GetScanModeAnsType(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x06, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00};
  command[7] = index & 0xFF;
  command[8] = index >> 8;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 12 && data.at(7) != 0x75)
  {
    throw std::runtime_error("GetScanModeAnsType: Unexpected data size");
  }
  co_return data.at(11);
}

boost::asio::awaitable<uint16_t> Config::GetScanModeTypical()
{
  // Return from 1 not 0x81
  std::vector<uint8_t> command = {0xA5, 0x84, 0x04, 0x7C, 0x00, 0x00, 0x00};
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 13 && data.at(7) != 0x7C)
  {
    throw std::runtime_error("GetScanModeTypical: Unexpected data size");
  }
  uint16_t typical = data.at(12) << 8 | data.at(11);
  co_return typical;
}

boost::asio::awaitable<std::string> Config::GetScanModeName(uint16_t index)
{
  std::vector<uint8_t> command = {0xA5, 0x84, 0x06, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00};
  command[7] = index & 0xFF;
  command[8] = index >> 8;
  command.push_back(GetCheckSum(command));
  std::vector<uint8_t> data = co_await serial_port_->WriteRead(command);
  if (data.size() != 13 && data.at(7) != 0x7F)
  {
    throw std::runtime_error("GetScanModeName: Unexpected data size");
  }
  std::string name;
  name.reserve(data.size() - 11);
  for (size_t i = 11; i < data.size(); i++)
  {
    name += (char)data[i];
  }
  co_return name;
}