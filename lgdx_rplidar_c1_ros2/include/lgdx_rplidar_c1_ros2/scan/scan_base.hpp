#ifndef SCAN_BASE_HPP
#define SCAN_BASE_HPP

#include <boost/asio.hpp>

#include "../structs.hpp"

class ScanBase
{
  public:
    virtual ~ScanBase() = default;
    virtual boost::asio::awaitable<void> Start() = 0;
    virtual boost::asio::awaitable<std::vector<LidarScanData>> GetData() = 0;
};

#endif // SCAN_BASE_HPP