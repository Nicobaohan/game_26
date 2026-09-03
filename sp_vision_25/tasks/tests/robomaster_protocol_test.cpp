#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include "io/gimbal/robomaster_protocol.hpp"

namespace
{
bool expect(bool condition, const char * message)
{
  if (!condition) std::cerr << "FAILED: " << message << '\n';
  return condition;
}
}  // namespace

int main()
{
  bool ok = true;

  io::VisionData vision;
  vision.id = 3;
  vision.mode = io::AUTO_AIM_MODE;
  vision.yaw = 1.25F;
  vision.yaw_vel = -0.5F;
  vision.pitch = 0.2F;
  vision.pitch_vel = 0.1F;
  vision.quaternion[0] = 1.0F;
  vision.quaternion[1] = 0.0F;
  vision.quaternion[2] = 0.0F;
  vision.quaternion[3] = 0.0F;
  vision.shoot_speed = 22.0F;
  vision.bullet_count = 17;

  auto bytes = io::robomaster_protocol::pack_frame(
    io::VISION_ID, &vision, sizeof(vision), 7);
  ok &= expect(
    bytes.size() == sizeof(vision) + io::robomaster_protocol::FRAME_FIXED_SIZE,
    "packed VisionData size");

  io::robomaster_protocol::Frame decoded;
  ok &= expect(
    io::robomaster_protocol::unpack_frame(bytes.data(), bytes.size(), decoded),
    "decode valid VisionData frame");
  ok &= expect(decoded.seq == 7, "sequence round trip");
  ok &= expect(decoded.cmd_id == io::VISION_ID, "command id round trip");
  ok &= expect(decoded.payload.size() == sizeof(vision), "payload size round trip");

  io::VisionData decoded_vision;
  std::memcpy(&decoded_vision, decoded.payload.data(), sizeof(decoded_vision));
  ok &= expect(decoded_vision.mode == io::AUTO_AIM_MODE, "mode round trip");
  ok &= expect(std::abs(decoded_vision.yaw - vision.yaw) < 1e-6F, "yaw round trip");
  ok &= expect(decoded_vision.bullet_count == vision.bullet_count, "bullet count round trip");

  auto bad_header = bytes;
  bad_header[1] ^= 0x01;
  ok &= expect(
    !io::robomaster_protocol::unpack_frame(bad_header.data(), bad_header.size(), decoded),
    "reject invalid CRC8");

  auto bad_payload = bytes;
  bad_payload[sizeof(io::FrameHeader) + sizeof(uint16_t)] ^= 0x01;
  ok &= expect(
    !io::robomaster_protocol::unpack_frame(bad_payload.data(), bad_payload.size(), decoded),
    "reject invalid CRC16");

  auto bad_end = bytes;
  bad_end.back() = 0;
  ok &= expect(
    !io::robomaster_protocol::unpack_frame(bad_end.data(), bad_end.size(), decoded),
    "reject invalid frame end");

  io::RobotCtrlData command;
  command.yaw = -0.3F;
  command.pitch = 0.15F;
  command.target_lock = io::TARGET_LOCKED;
  command.fire_command = 0;
  auto command_bytes = io::robomaster_protocol::pack_frame(
    io::CHASSIS_CTRL_CMD_ID, &command, sizeof(command), 8);
  ok &= expect(
    io::robomaster_protocol::unpack_frame(
      command_bytes.data(), command_bytes.size(), decoded),
    "decode valid RobotCtrlData frame");
  ok &= expect(decoded.cmd_id == io::CHASSIS_CTRL_CMD_ID, "control command id round trip");

  if (!ok) return 1;
  std::cout << "RoboMaster protocol tests passed.\n";
  return 0;
}
