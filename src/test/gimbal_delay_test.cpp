// Gimbal delay test - serial port version (no CAN).
// Ported from sp_vision_25/tests/gimbal_delay/gimbal_delay_test.cpp:
//   io::CBoard (SocketCAN)  ->  SerialMain (/dev/robomaster, protocol_new.hpp)
// Read:  VisionData quaternion (w,x,y,z) -> ZYX euler yaw/pitch
// Send:  RobotCtrlData yaw/pitch target in DEGREES (MCU wire units),
//         optional target_lock / fire_command.
//
// Build:  see CMakeLists.txt in this directory
// Run:    ./build/gimbal_delay_test --help
// NOTE:   this test MOVES THE GIMBAL. Make sure the robot is safe before running.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "serial_main.h"

using namespace std::chrono_literals;

namespace
{

constexpr double kRad2Deg = 57.2957795131;

struct TestResult
{
  double yaw_cmd_deg = 0.0;
  double pitch_cmd_deg = 0.0;
  double delay_ms = 0.0;
  double yaw_err_deg = 0.0;
  double pitch_err_deg = 0.0;
};

// VisionData stream rate is ~500 Hz on this robot. ReceiverMain() reads ONE
// frame per call, so to observe the LATEST attitude we drain several frames
// per poll; otherwise stale backlog inflates the measured delay.
int drain_frames_for(int poll_interval_ms)
{
  return std::max(2, poll_interval_ms / 2 + 2);
}

// Reads the freshest VisionData from the serial port and returns
// {yaw_deg, pitch_deg} from the onboard quaternion (w, x, y, z),
// using the same ZYX convention as tools::eulers(q, 2, 1, 0).
std::array<double, 2> read_current_angles(SerialMain & serial, int drain_frames)
{
  for (int i = 0; i < drain_frames; ++i) {
    // Parse failures are tolerated: vision_msg_ keeps the last good frame.
    serial.ReceiverMain();
  }
  const auto & q = serial.vision_msg_.quaternion;  // w, x, y, z
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  const double pitch =
    std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  return {yaw * kRad2Deg, pitch * kRad2Deg};
}

double percentile(const std::vector<double> & values, double p)
{
  if (values.empty()) return 0.0;
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const double idx =
    std::clamp(p * (sorted.size() - 1), 0.0, static_cast<double>(sorted.size() - 1));
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = static_cast<size_t>(std::ceil(idx));
  if (lo == hi) return sorted[lo];
  const double w = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - w) + sorted[hi] * w;
}

// Sends a gimbal target. MCU wire units are DEGREES (protocol_new.hpp:
// "absolute angles in degrees"; robot_ctrl_node multiplies ROS radians by
// RAD_TO_DEG before writing). Optional target lock; fire mirrors the
// robot_ctrl_node gate (fire only honored while locked).
bool send_step(SerialMain & serial, double yaw_deg, double pitch_deg, bool lock = false, bool fire = false)
{
  io::RobotCtrlData data;
  data.yaw = static_cast<float>(yaw_deg);    // degrees on wire - NOT radians
  data.pitch = static_cast<float>(pitch_deg);
  data.target_lock = lock ? io::TARGET_LOCKED : io::TARGET_UNLOCKED;
  data.fire_command = (lock && fire) ? 1 : 0;
  return serial.SenderMain(data);
}

bool reached_target(
  const std::array<double, 2> & current, const std::array<double, 2> & target, double tol_deg)
{
  const double yaw_err = std::abs(current[0] - target[0]);
  const double pitch_err = std::abs(current[1] - target[1]);
  return yaw_err <= tol_deg && pitch_err <= tol_deg;
}

void print_summary(const std::vector<TestResult> & results)
{
  std::vector<double> delays;
  delays.reserve(results.size());
  for (const auto & r : results) delays.push_back(r.delay_ms);

  const double mean =
    std::accumulate(delays.begin(), delays.end(), 0.0) / std::max<size_t>(1, delays.size());
  const double p90 = percentile(delays, 0.90);
  const double p95 = percentile(delays, 0.95);
  const double max_v = *std::max_element(delays.begin(), delays.end());
  const double min_v = *std::min_element(delays.begin(), delays.end());

  std::cout << "[GimbalDelayTest] total_runs=" << results.size() << " mean_ms=" << std::fixed
            << std::setprecision(2) << mean << " p90_ms=" << p90 << " p95_ms=" << p95
            << " min_ms=" << min_v << " max_ms=" << max_v << std::endl;
}

}  // namespace

int main(int argc, char * argv[])
{
  std::string device = "/dev/robomaster";
  double yaw_step_deg = 15.0;
  double pitch_step_deg = 10.0;
  int iterations = 10;
  double tolerance_deg = 1.0;
  double timeout_s = 3.0;
  std::string log_path = "gimbal_delay_log.csv";
  std::string trace_path;  // optional per-poll trajectory log
  int poll_interval_ms = 10;
  bool use_lock = false;
  bool use_fire = false;  // DANGER: launches projectiles; requires --lock

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next_value = [&]() -> std::string {
      if (i + 1 < argc) return argv[++i];
      throw std::runtime_error("Missing value for argument: " + arg);
    };

    if (arg == "--help" || arg == "-h") {
      std::cout
        << "Usage: gimbal_delay_test [--device path] [--yaw-step deg] [--pitch-step deg]"
        << " [--iterations N] [--tolerance deg] [--timeout s] [--log path] [--poll-interval ms]"
        << " [--lock] [--trace path] [--fire]" << std::endl
        << "  --lock : set target_lock=LOCKED(49) on step frames" << std::endl
        << "  --fire : DANGER - set fire_command=1 on step frames (ignored without --lock;"
        << " the robot WILL SHOOT)" << std::endl;
      return 0;
    }
    if (arg == "--device" || arg == "-d")
      device = next_value();
    else if (arg == "--yaw-step" || arg == "-y")
      yaw_step_deg = std::stod(next_value());
    else if (arg == "--pitch-step" || arg == "-p")
      pitch_step_deg = std::stod(next_value());
    else if (arg == "--iterations" || arg == "-n")
      iterations = std::max(1, std::stoi(next_value()));
    else if (arg == "--tolerance" || arg == "-t")
      tolerance_deg = std::stod(next_value());
    else if (arg == "--timeout" || arg == "-s")
      timeout_s = std::stod(next_value());
    else if (arg == "--log" || arg == "-l")
      log_path = next_value();
    else if (arg == "--poll-interval" || arg == "-i")
      poll_interval_ms = std::max(1, std::stoi(next_value()));
    else if (arg == "--lock")
      use_lock = true;
    else if (arg == "--fire")
      use_fire = true;
    else if (arg == "--trace")
      trace_path = next_value();
    else {
      std::cout << "Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  try {
    SerialMain serial(device);

    // Sanity check: the board must actually deliver at least one VisionData
    // frame before we command anything. (ReceiverMain() returns false right
    // away when the serial port failed to open, so a dead/absent device is
    // detected here instead of silently producing all-timeout runs.)
    std::array<double, 2> initial{0.0, 0.0};
    bool got_frame = false;
    const auto probe_deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < probe_deadline) {
      if (serial.ReceiverMain()) {
        got_frame = true;
        initial = read_current_angles(serial, 8);
        break;
      }
      std::this_thread::sleep_for(20ms);
    }
    if (!got_frame) {
      std::cerr << "[GimbalDelayTest] no VisionData received from " << device
                << " (is the board powered & connected? aborting)" << std::endl;
      return 1;
    }
    std::cout << "[GimbalDelayTest] device=" << device << " initial yaw=" << initial[0]
              << "deg pitch=" << initial[1] << "deg" << std::endl;

    const int drain_frames = drain_frames_for(poll_interval_ms);
    std::cout << "[GimbalDelayTest] yaw_step=" << yaw_step_deg << "deg pitch_step=" << pitch_step_deg
              << "deg iterations=" << iterations << " poll=" << poll_interval_ms
              << "ms drain=" << drain_frames << " frames/poll" << std::endl;

    std::vector<TestResult> results;
    results.reserve(static_cast<size_t>(iterations));

    std::ofstream trace;
    if (!trace_path.empty()) {
      trace.open(trace_path);
      trace << "t_ms,phase,cmd_yaw_deg,cmd_pitch_deg,cur_yaw_deg,cur_pitch_deg\n";
    }

    for (int i = 0; i < iterations; ++i) {
      const std::array<double, 2> target = {yaw_step_deg, pitch_step_deg};
      const auto start_ts = std::chrono::steady_clock::now();

      bool reached = false;
      auto deadline = start_ts + std::chrono::duration<double>(timeout_s);
      std::array<double, 2> current{0.0, 0.0};
      std::array<double, 2> last_error{0.0, 0.0};
      int send_fails = 0;

      while (std::chrono::steady_clock::now() < deadline) {
        // STREAM the setpoint every poll cycle (~100 Hz): the firmware expects
        // continuous control frames (robot_ctrl_node re-sends on every
        // /Robot_ctrl_data message); a single one-shot frame gets ignored.
        if (!send_step(serial, yaw_step_deg, pitch_step_deg, use_lock, use_fire)) ++send_fails;
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
        current = read_current_angles(serial, drain_frames);
        last_error = {std::abs(current[0] - target[0]), std::abs(current[1] - target[1])};
        if (trace.is_open()) {
          const double t_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_ts).count();
          trace << t_ms << ",step," << yaw_step_deg << ',' << pitch_step_deg << ','
                << current[0] << ',' << current[1] << '\n';
        }
        if (reached_target(current, target, tolerance_deg)) {
          reached = true;
          break;
        }
      }

      const auto finish_ts = std::chrono::steady_clock::now();
      const double delay_ms =
        std::chrono::duration<double, std::milli>(finish_ts - start_ts).count();

      TestResult result;
      result.yaw_cmd_deg = yaw_step_deg;
      result.pitch_cmd_deg = pitch_step_deg;
      result.delay_ms = delay_ms;
      result.yaw_err_deg = last_error[0];
      result.pitch_err_deg = last_error[1];
      results.push_back(result);

      if (!reached) {
        std::cout << "[GimbalDelayTest] run " << i + 1 << " TIMEOUT"
                  << " yaw_err=" << last_error[0] << "deg pitch_err=" << last_error[1]
                  << "deg delay_ms=" << delay_ms << std::endl;
      } else {
        std::cout << "[GimbalDelayTest] run " << i + 1 << " reach_ok delay_ms=" << delay_ms
                  << " yaw_err=" << last_error[0] << "deg pitch_err=" << last_error[1] << "deg"
                  << std::endl;
      }
      if (send_fails > 0) {
        std::cerr << "[GimbalDelayTest] run " << i + 1 << ": " << send_fails
                  << " send failures" << std::endl;
      }

      // Stream the home setpoint briefly so the gimbal returns, then pause.
      const auto home_start = std::chrono::steady_clock::now();
      for (int h = 0; h < 50; ++h) {
        send_step(serial, 0.0, 0.0);
        std::this_thread::sleep_for(10ms);
        if (trace.is_open()) {
          const auto cur = read_current_angles(serial, 4);
          const double t_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - home_start).count();
          trace << t_ms << ",home,0,0," << cur[0] << ',' << cur[1] << '\n';
        }
      }
      std::this_thread::sleep_for(300ms);
    }

    print_summary(results);

    std::ofstream csv(log_path);
    csv << "run,yaw_cmd_deg,pitch_cmd_deg,delay_ms,yaw_err_deg,pitch_err_deg\n";
    for (size_t i = 0; i < results.size(); ++i) {
      const auto & r = results[i];
      csv << i + 1 << ',' << r.yaw_cmd_deg << ',' << r.pitch_cmd_deg << ',' << r.delay_ms << ','
          << r.yaw_err_deg << ',' << r.pitch_err_deg << '\n';
    }
    csv.close();

    std::cout << "[GimbalDelayTest] CSV saved to " << log_path << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "[GimbalDelayTest] failed: " << e.what() << std::endl;
    return 1;
  }
}
