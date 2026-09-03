#include "gimbal.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  const auto com_port = tools::read<std::string>(yaml, "com_port");
  const auto baudrate = yaml["baudrate"] ? yaml["baudrate"].as<uint32_t>() : 115200U;

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    auto timeout = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (serial_.isOpen()) serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(const RobotCtrlData & data)
{
  auto frame = robomaster_protocol::pack_frame(
    CHASSIS_CTRL_CMD_ID, &data, sizeof(data), tx_seq_++);

  try {
    const auto written = serial_.write(frame.data(), frame.size());
    if (written != frame.size()) {
      tools::logger()->warn("[Gimbal] Short serial write: {}/{} bytes", written, frame.size());
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  RobotCtrlData data;
  data.yaw = yaw;
  data.yaw_vel = yaw_vel;
  data.yaw_acc = yaw_acc;
  data.pitch = pitch;
  data.pitch_vel = pitch_vel;
  data.pitch_acc = pitch_acc;
  data.target_lock = control ? TARGET_LOCKED : TARGET_UNLOCKED;
  data.fire_command = control && fire ? 1 : 0;
  send(data);
}

bool Gimbal::read_exact(uint8_t * buffer, size_t size)
{
  std::size_t offset = 0;
  try {
    while (!quit_ && offset < size) {
      const auto count = serial_.read(buffer + offset, size - offset);
      if (count == 0) return false;
      offset += count;
    }
  } catch (const std::exception &) {
    return false;
  }
  return offset == size;
}

bool Gimbal::read_frame(robomaster_protocol::Frame & frame)
{
  uint8_t sof = 0;
  while (!quit_) {
    if (!read_exact(&sof, sizeof(sof))) return false;
    if (sof == HEADER_SOF) break;
  }
  if (quit_) return false;

  std::vector<uint8_t> bytes(sizeof(FrameHeader), 0);
  bytes[0] = sof;
  if (!read_exact(bytes.data() + 1, sizeof(FrameHeader) - 1)) return false;

  FrameHeader header;
  std::memcpy(&header, bytes.data(), sizeof(header));
  if (!tools::check_crc8(bytes.data(), sizeof(header))) return false;
  if (header.data_length > robomaster_protocol::MAX_PAYLOAD_SIZE) return false;

  const auto body_size =
    sizeof(uint16_t) + header.data_length + sizeof(uint16_t) + sizeof(MsgEndInfo);
  bytes.resize(sizeof(FrameHeader) + body_size);
  if (!read_exact(bytes.data() + sizeof(FrameHeader), body_size)) return false;

  return robomaster_protocol::unpack_frame(bytes.data(), bytes.size(), frame);
}

void Gimbal::update_state(
  const VisionData & data, std::chrono::steady_clock::time_point timestamp)
{
  Eigen::Quaterniond q(
    data.quaternion[0], data.quaternion[1], data.quaternion[2], data.quaternion[3]);
  const auto squared_norm = q.squaredNorm();
  if (!std::isfinite(squared_norm) || std::abs(squared_norm - 1.0) > 0.1) {
    tools::logger()->warn("[Gimbal] Invalid quaternion norm: {:.4f}", squared_norm);
    return;
  }
  queue_.push({q.normalized(), timestamp});

  std::lock_guard<std::mutex> lock(mutex_);
  state_.yaw = data.yaw;
  state_.yaw_vel = data.yaw_vel;
  state_.pitch = data.pitch;
  state_.pitch_vel = data.pitch_vel;
  state_.bullet_speed = data.shoot_speed;
  state_.bullet_count = data.bullet_count;
  mode_ = data.mode == AUTO_AIM_MODE ? GimbalMode::AUTO_AIM : GimbalMode::IDLE;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    robomaster_protocol::Frame frame;
    if (!read_frame(frame)) {
      if (++error_count > 50) {
        error_count = 0;
        tools::logger()->warn("[Gimbal] Too many read errors, attempting to reconnect...");
        reconnect();
      }
      continue;
    }

    if (frame.cmd_id != VISION_ID || frame.payload.size() != sizeof(VisionData)) continue;

    VisionData data;
    std::memcpy(&data, frame.payload.data(), sizeof(data));
    update_state(data, std::chrono::steady_clock::now());
    error_count = 0;
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  constexpr int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      if (serial_.isOpen()) serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      serial_.open();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
