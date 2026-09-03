#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0},
  last_fire_time_(std::chrono::steady_clock::now()),
  last_valid_target_time_(std::chrono::steady_clock::now()),
  stable_target_frames_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();

  fire_cooldown_ = yaml["fire_cooldown"].IsDefined() ? yaml["fire_cooldown"].as<double>() : 0.25;
  min_target_lock_frames_ =
    yaml["min_target_lock_frames"].IsDefined() ? yaml["min_target_lock_frames"].as<int>() : 5;
  max_sensor_stale_time_ =
    yaml["max_sensor_stale_time"].IsDefined() ? yaml["max_sensor_stale_time"].as<double>() : 0.05;
  max_pnp_residual_ =
    yaml["max_pnp_residual"].IsDefined() ? yaml["max_pnp_residual"].as<double>() : 1.5;
  max_track_uncertainty_ =
    yaml["max_track_uncertainty"].IsDefined() ? yaml["max_track_uncertainty"].as<double>() : 0.2;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target = targets.front();
  auto target_x = target.ekf_x()[0];
  auto target_y = target.ekf_x()[2];
  auto distance = std::sqrt(tools::square(target_x) + tools::square(target_y));
  auto tolerance = distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

  const auto now = std::chrono::steady_clock::now();
  const double elapsed_since_last_fire =
    std::chrono::duration<double>(now - last_fire_time_).count();
  const double elapsed_since_last_valid_target =
    std::chrono::duration<double>(now - last_valid_target_time_).count();
  const bool cooldown_ok = elapsed_since_last_fire >= fire_cooldown_;
  const bool aim_point_valid = aimer.debug_aim_point.valid;
  const bool target_valid =
    aim_point_valid && distance > 0.1 && distance < 15.0 && !target.diverged();
  const bool sensor_fresh =
    max_sensor_stale_time_ <= 0.0 || elapsed_since_last_valid_target <= max_sensor_stale_time_;
  const bool stable_motion =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2.0 &&
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance;

  if (target_valid) {
    last_valid_target_time_ = now;
  }

  const double target_speed = std::abs(target.ekf_x()[7]);
  const double target_radius = std::max(std::abs(target.ekf_x()[8]), 1e-6);
  const bool target_not_noisy = target_radius < max_track_uncertainty_ * 10.0 && target_speed < 8.0;

  const bool allow_fire = target_valid && sensor_fresh && stable_motion && cooldown_ok &&
                          target_not_noisy;

  if (allow_fire) {
    stable_target_frames_++;
  } else {
    stable_target_frames_ = 0;
  }

  const bool locked = stable_target_frames_ >= min_target_lock_frames_;
  if (!locked) {
    last_command_ = command;
    return false;
  }

  if (allow_fire) {
    last_fire_time_ = now;
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim