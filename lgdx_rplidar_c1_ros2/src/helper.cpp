#include "lgdx_rplidar_c1_ros2/helper.hpp"

#include <format>
#include <iostream>

void Helper::PrintHex(const std::vector<uint8_t> &data)
{
  std::string str;
  for (auto d : data)
  {
    str += std::format("{:02X} ", d);
  }
  std::cout << str << std::endl;
}