#ifndef SCAN_HPP
#define SCAN_HPP

#include <boost/circular_buffer.hpp>

#include "scan_base.hpp"
#include "../serial_port.hpp"
#include "../structs.hpp"

class Scan : public ScanBase
{
  public:
    Scan(std::shared_ptr<SerialPort> serial_port);
    
    boost::asio::awaitable<void> Start() override;
    boost::asio::awaitable<std::vector<LidarScanData>> GetData() override;

  private:
    const size_t kBufferSize = 1024;
    const size_t kDescriptorSize = 7;
    const size_t kScanDataSize = 5;

    std::shared_ptr<SerialPort> serial_port_;
    boost::circular_buffer<uint8_t> buffer_;
};

#endif // SCAN_HPP