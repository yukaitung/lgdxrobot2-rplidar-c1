#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include <cstdint>

typedef struct 
{
  uint8_t header[7];
  uint8_t model;
  uint8_t firmware_minor;
  uint8_t firmware_major;
  uint8_t hardware;
  uint8_t serial_number[16];
} LidarInfo;

typedef struct
{
  uint8_t header[7];
  uint8_t status;
  uint16_t error_code;
} LidarHealth;

typedef struct
{
  uint8_t header[7];
  uint16_t time_standard;
  uint16_t time_express;
} LidarSampleRate;

#endif // STRUCTS_HPP