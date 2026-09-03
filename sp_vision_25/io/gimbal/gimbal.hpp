#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "robomaster_protocol.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(const RobotCtrlData & data);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  uint8_t tx_seq_ = 0;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read_exact(uint8_t * buffer, size_t size);
  bool read_frame(robomaster_protocol::Frame & frame);
  void update_state(const VisionData & data, std::chrono::steady_clock::time_point timestamp);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
