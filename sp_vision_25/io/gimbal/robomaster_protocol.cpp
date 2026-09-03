#include "robomaster_protocol.hpp"

#include <cstring>
#include <stdexcept>

#include "tools/crc.hpp"

namespace io::robomaster_protocol
{

std::vector<uint8_t> pack_frame(
  uint16_t cmd_id, const void * payload, uint16_t payload_size, uint8_t seq)
{
  if (payload_size > MAX_PAYLOAD_SIZE) {
    throw std::length_error("RoboMaster payload is too large");
  }
  if (payload_size > 0 && payload == nullptr) {
    throw std::invalid_argument("RoboMaster payload is null");
  }

  const auto crc16_offset = sizeof(FrameHeader) + sizeof(cmd_id) + payload_size;
  std::vector<uint8_t> bytes(crc16_offset + sizeof(uint16_t) + sizeof(MsgEndInfo), 0);

  FrameHeader header;
  header.data_length = payload_size;
  header.seq = seq;
  header.crc8 = tools::get_crc8(
    reinterpret_cast<const uint8_t *>(&header), sizeof(header) - sizeof(header.crc8));

  std::size_t offset = 0;
  std::memcpy(bytes.data() + offset, &header, sizeof(header));
  offset += sizeof(header);
  std::memcpy(bytes.data() + offset, &cmd_id, sizeof(cmd_id));
  offset += sizeof(cmd_id);
  if (payload_size > 0) std::memcpy(bytes.data() + offset, payload, payload_size);

  const auto crc16 = tools::get_crc16(bytes.data(), crc16_offset);
  bytes[crc16_offset] = static_cast<uint8_t>(crc16 & 0xff);
  bytes[crc16_offset + 1] = static_cast<uint8_t>((crc16 >> 8) & 0xff);

  const MsgEndInfo end;
  std::memcpy(bytes.data() + crc16_offset + sizeof(crc16), &end, sizeof(end));
  return bytes;
}

bool unpack_frame(const uint8_t * data, std::size_t size, Frame & frame)
{
  if (data == nullptr || size < FRAME_FIXED_SIZE) return false;

  FrameHeader header;
  std::memcpy(&header, data, sizeof(header));
  if (header.sof != HEADER_SOF) return false;
  if (!tools::check_crc8(data, sizeof(FrameHeader))) return false;
  if (header.data_length > MAX_PAYLOAD_SIZE) return false;

  const auto expected_size = FRAME_FIXED_SIZE + header.data_length;
  if (size != expected_size) return false;

  const auto crc16_size = size - sizeof(MsgEndInfo);
  if (!tools::check_crc16(data, crc16_size)) return false;

  MsgEndInfo end;
  std::memcpy(&end, data + size - sizeof(end), sizeof(end));
  if (end.end1 != END1_SOF || end.end2 != END2_SOF) return false;

  frame.seq = header.seq;
  std::memcpy(&frame.cmd_id, data + sizeof(FrameHeader), sizeof(frame.cmd_id));
  const auto * payload_begin = data + sizeof(FrameHeader) + sizeof(frame.cmd_id);
  frame.payload.assign(payload_begin, payload_begin + header.data_length);
  return true;
}

}  // namespace io::robomaster_protocol
