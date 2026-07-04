#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "serial_port.hpp"
#include "structs.hpp"

class Config
{
  public:
    Config(std::shared_ptr<SerialPort> serial_port);
    boost::asio::awaitable<LidarInfo> GetInfo();
    boost::asio::awaitable<LidarHealth> GetHealth();
    boost::asio::awaitable<LidarSampleRate> GetSampleRate();
    boost::asio::awaitable<uint16_t> GetScanModeCount();
    boost::asio::awaitable<uint32_t> GetScanModeUsPerSample(uint16_t index);
    boost::asio::awaitable<uint32_t> GetScanModeMaxDistance(uint16_t index);
    boost::asio::awaitable<uint8_t> GetScanModeAnsType(uint16_t index);
    boost::asio::awaitable<uint16_t> GetScanModeTypical();
    boost::asio::awaitable<std::string> GetScanModeName(uint16_t index);

  private:
    std::shared_ptr<SerialPort> serial_port_;
    uint8_t GetCheckSum(const std::vector<uint8_t> &data);
};

#endif // CONFIG_HPP
