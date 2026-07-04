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

typedef struct
{
  int mode;
  float sample_rate; // KHz
  uint32_t max_distance; // m
  uint8_t answer_type;
} LidarScanMode;

typedef struct
{
  bool is_new_scan;
  uint8_t quality;
  float angle; // degree
  float distance; // mm
} LidarScanData;

#endif // STRUCTS_HPP