#ifndef SCAN_HPP
#define SCAN_HPP

#include <boost/circular_buffer.hpp>

#include "serial_port.hpp"
#include "structs.hpp"

class Scan
{
  public:
    Scan(std::shared_ptr<SerialPort> serial_port);
    
    boost::asio::awaitable<void> StartNormalScan();
    boost::asio::awaitable<std::vector<LidarScanData>> NormalScan();

  private:
    const size_t kBufferSize = 1024;
    const size_t kDescriptorSize = 7;
    const size_t kScanDataSize = 5;

    std::shared_ptr<SerialPort> serial_port_;
    boost::circular_buffer<uint8_t> buffer_;
};

#endif // SCAN_HPP