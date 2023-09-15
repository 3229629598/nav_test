// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace data_process_ns
{
struct rx_bag
{
  uint8_t header = 0x5A;
  uint8_t nav_request;
  uint16_t checksum;
} __attribute__((packed));

struct tx_bag
{
  uint8_t header = 0xA5;
  float vx;
  float vy;
  float wz;
  uint16_t checksum;
} __attribute__((packed));

#define tx_len sizeof(tx_bag)
#define rx_len sizeof(rx_bag)

inline rx_bag fromVector(const std::vector<uint8_t> & data)
{
  rx_bag packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const tx_bag & data)
{
  std::vector<uint8_t> packet(tx_len);
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + tx_len, packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
