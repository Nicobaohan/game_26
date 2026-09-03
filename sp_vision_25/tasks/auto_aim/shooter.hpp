#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;

  // Fire decision / safety gate parameters.
  double fire_cooldown_;
  int min_target_lock_frames_;
  double max_sensor_stale_time_;
  double max_pnp_residual_;
  double max_track_uncertainty_;

  std::chrono::steady_clock::time_point last_fire_time_;
  std::chrono::steady_clock::time_point last_valid_target_time_;
  int stable_target_frames_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP