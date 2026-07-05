#ifndef HELPER_HPP
#define HELPER_HPP

#include <vector>
#include <cstdint>

class Helper
{
  public:
    static void PrintHex(const std::vector<uint8_t> &data);
    static float DegToRad(float deg);
};

#endif // HELPER_HPP