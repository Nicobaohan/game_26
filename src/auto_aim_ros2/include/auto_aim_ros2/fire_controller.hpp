#ifndef AUTO_AIM_ROS2__FIRE_CONTROLLER_HPP_
#define AUTO_AIM_ROS2__FIRE_CONTROLLER_HPP_

#include <chrono>

namespace auto_aim_ros2
{

struct FireControllerConfig
{
  double hold_s = 0.15;
  double interval_s = 0.5;
  double shot_period_s = 0.1;
  int burst_count = 3;
};

// Produces one fire pulse per requested shot. Any failed safety gate cancels
// the active burst immediately and a new hold period is required to restart.
class FireController
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit FireController(FireControllerConfig config) : config_(config) {}

  int update(bool eligible, TimePoint now)
  {
    if (!eligible)
    {
      eligible_since_ = TimePoint{};
      burst_active_ = false;
      shots_remaining_ = 0;
      pulse_sent_last_update_ = false;
      return 0;
    }

    if (eligible_since_ == TimePoint{})
    {
      eligible_since_ = now;
    }

    // Always publish at least one zero between fire pulses. This preserves
    // rising edges even when the image callback is slower than shot_period_s.
    if (pulse_sent_last_update_)
    {
      pulse_sent_last_update_ = false;
      return 0;
    }

    if (burst_active_)
    {
      return now >= next_shot_at_ ? emit_shot(now) : 0;
    }

    const bool held_long_enough =
        seconds_since(eligible_since_, now) >= config_.hold_s;
    const bool interval_complete =
        next_burst_allowed_at_ == TimePoint{} || now >= next_burst_allowed_at_;
    if (!held_long_enough || !interval_complete)
    {
      return 0;
    }

    burst_active_ = true;
    shots_remaining_ = config_.burst_count;
    return emit_shot(now);
  }

private:
  static double seconds_since(TimePoint start, TimePoint now)
  {
    return std::chrono::duration<double>(now - start).count();
  }

  TimePoint after(TimePoint now, double seconds) const
  {
    return now + std::chrono::duration_cast<Clock::duration>(
                     std::chrono::duration<double>(seconds));
  }

  int emit_shot(TimePoint now)
  {
    --shots_remaining_;
    pulse_sent_last_update_ = true;
    // Keep the cooldown even if the remainder of this burst is cancelled.
    next_burst_allowed_at_ = after(now, config_.interval_s);
    if (shots_remaining_ > 0)
    {
      next_shot_at_ = after(now, config_.shot_period_s);
    }
    else
    {
      burst_active_ = false;
    }
    return 1;
  }

  FireControllerConfig config_;
  TimePoint eligible_since_{};
  TimePoint next_shot_at_{};
  TimePoint next_burst_allowed_at_{};
  bool burst_active_ = false;
  bool pulse_sent_last_update_ = false;
  int shots_remaining_ = 0;
};

}  // namespace auto_aim_ros2

#endif  // AUTO_AIM_ROS2__FIRE_CONTROLLER_HPP_
