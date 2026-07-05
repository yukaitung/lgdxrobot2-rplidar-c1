#ifndef EXPRESS_SCAN_HPP
#define EXPRESS_SCAN_HPP

#include <boost/circular_buffer.hpp>

#include "scan_base.hpp"
#include "../serial_port.hpp"
#include "../structs.hpp"

class ExpressScan : public ScanBase
{
  public:
    ExpressScan(std::shared_ptr<SerialPort> serial_port, uint8_t mode);

    boost::asio::awaitable<void> Start() override;
    boost::asio::awaitable<std::vector<LidarScanData>> GetData() override;

  private:
    const size_t kBufferSize = 1024;
    const size_t kDescriptorSize = 7;
    const size_t kScanDataSize = 84;
    
    uint8_t mode_ = 0;

    std::shared_ptr<SerialPort> serial_port_;
    boost::circular_buffer<uint8_t> buffer_;

    float AngleDiff(float this_angle, float next_angle);
};

#endif // EXPRESS_SCAN_HPP