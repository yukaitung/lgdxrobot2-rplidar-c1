#include <format>
#include <iostream>
#include <numbers>

#include "lgdx_rplidar_c1_ros2/helper.hpp"

void Helper::PrintHex(const std::vector<uint8_t> &data)
{
  std::string str;
  for (auto d : data)
  {
    str += std::format("{:02X} ", d);
  }
  std::cout << str << std::endl;
}

float Helper::DegToRad(float deg)
{
  return deg * (std::numbers::pi_v<float> / 180.0f);
}