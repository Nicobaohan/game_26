#ifndef IO__ROBOMASTER_PROTOCOL_HPP
#define IO__ROBOMASTER_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "protocol_new.hpp"

namespace io::robomaster_protocol
{

constexpr std::size_t FRAME_FIXED_SIZE =
  sizeof(FrameHeader) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(MsgEndInfo);
constexpr std::size_t MAX_PAYLOAD_SIZE = 256;

struct Frame
{
  uint8_t seq = 0;
  uint16_t cmd_id = 0;
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> pack_frame(
  uint16_t cmd_id, const void * payload, uint16_t payload_size, uint8_t seq);

bool unpack_frame(const uint8_t * data, std::size_t size, Frame & frame);

}  // namespace io::robomaster_protocol

#endif  // IO__ROBOMASTER_PROTOCOL_HPP
