#ifndef HELPER_HPP
#define HELPER_HPP

#include <vector>
#include <cstdint>

class Helper
{
  public:
    static void PrintHex(const std::vector<uint8_t> &data);
};

#endif // HELPER_HPP