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

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

namespace {

struct TestResult
{
  double yaw_cmd_deg = 0.0;
  double pitch_cmd_deg = 0.0;
  double delay_ms = 0.0;
  double yaw_err_deg = 0.0;
  double pitch_err_deg = 0.0;
};

std::array<double, 2> read_current_angles(io::CBoard & cboard)
{
  const auto ts = std::chrono::steady_clock::now();
  const auto q = cboard.imu_at(ts);
  const auto eulers = tools::eulers(q, 2, 1, 0);
  return {eulers[0] * 57.2957795131, eulers[1] * 57.2957795131};
}

double to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

double percentile(const std::vector<double> & values, double p)
{
  if (values.empty()) return 0.0;
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const double idx = std::clamp(p * (sorted.size() - 1), 0.0, static_cast<double>(sorted.size() - 1));
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = static_cast<size_t>(std::ceil(idx));
  if (lo == hi) return sorted[lo];
  const double w = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - w) + sorted[hi] * w;
}

void send_step(io::CBoard & cboard, double yaw_deg, double pitch_deg)
{
  io::Command cmd;
  cmd.control = true;
  cmd.shoot = false;
  cmd.yaw = to_rad(yaw_deg);
  cmd.pitch = to_rad(pitch_deg);
  cboard.send(cmd);
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

  const double mean = std::accumulate(delays.begin(), delays.end(), 0.0) / std::max(1ul, delays.size());
  const double p90 = percentile(delays, 0.90);
  const double p95 = percentile(delays, 0.95);
  const double max_v = *std::max_element(delays.begin(), delays.end());
  const double min_v = *std::min_element(delays.begin(), delays.end());

  tools::logger()->info("[GimbalDelayTest] total_runs={} mean_ms={:.2f} p90_ms={:.2f} p95_ms={:.2f} min_ms={:.2f} max_ms={:.2f}",
    results.size(), mean, p90, p95, min_v, max_v);
}

}  // namespace

std::string resolve_config_path(const std::string & input)
{
  std::filesystem::path p(input);
  if (std::filesystem::exists(p)) return std::filesystem::absolute(p).string();

  const std::array<std::string, 4> candidates = {
    input,
    std::filesystem::path("../") / input,
    std::filesystem::path("./") / input,
    std::filesystem::path("/home/hero/game_26_current/sp_vision_25") / input};

  for (const auto & candidate : candidates) {
    std::filesystem::path cp(candidate);
    if (std::filesystem::exists(cp)) {
      return std::filesystem::absolute(cp).string();
    }
  }

  return input;
}

int main(int argc, char * argv[])
{
  std::string config_path = "../configs/sentry.yaml";
  double yaw_step_deg = 15.0;
  double pitch_step_deg = 10.0;
  int iterations = 10;
  double tolerance_deg = 1.0;
  double timeout_s = 3.0;
  std::string log_path = "gimbal_delay_log.csv";
  int poll_interval_ms = 10;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next_value = [&]() -> std::string {
      if (i + 1 < argc) return argv[++i];
      throw std::runtime_error("Missing value for argument: " + arg);
    };

    if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: gimbal_delay_test [--config path] [--yaw-step deg] [--pitch-step deg]"
                   << " [--iterations N] [--tolerance deg] [--timeout s] [--log path] [--poll-interval ms]"
                   << std::endl;
      return 0;
    }
    if (arg == "--config" || arg == "-c") config_path = resolve_config_path(next_value());
    else if (arg == "--yaw-step" || arg == "-y") yaw_step_deg = std::stod(next_value());
    else if (arg == "--pitch-step" || arg == "-p") pitch_step_deg = std::stod(next_value());
    else if (arg == "--iterations" || arg == "-n") iterations = std::max(1, std::stoi(next_value()));
    else if (arg == "--tolerance" || arg == "-t") tolerance_deg = std::stod(next_value());
    else if (arg == "--timeout" || arg == "-s") timeout_s = std::stod(next_value());
    else if (arg == "--log" || arg == "-l") log_path = next_value();
    else if (arg == "--poll-interval" || arg == "-i") poll_interval_ms = std::max(1, std::stoi(next_value()));
    else {
      std::cout << "Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  const auto resolved_path = resolve_config_path(config_path);
  if (!std::filesystem::exists(resolved_path)) {
    std::cerr << "Config file not found: " << resolved_path << std::endl;
    return 1;
  }

  try {
    io::CBoard cboard(resolved_path);

    std::vector<TestResult> results;
    results.reserve(static_cast<size_t>(iterations));

    for (int i = 0; i < iterations; ++i) {
      const std::array<double, 2> target = {yaw_step_deg, pitch_step_deg};
      const auto start_ts = std::chrono::steady_clock::now();
      send_step(cboard, yaw_step_deg, pitch_step_deg);

      bool reached = false;
      auto deadline = start_ts + std::chrono::duration<double>(timeout_s);
      std::array<double, 2> current{0.0, 0.0};
      std::array<double, 2> last_error{0.0, 0.0};

      while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
        current = read_current_angles(cboard);
        last_error = {std::abs(current[0] - target[0]), std::abs(current[1] - target[1])};
        if (reached_target(current, target, tolerance_deg)) {
          reached = true;
          break;
        }
      }

      const auto finish_ts = std::chrono::steady_clock::now();
      const double delay_ms = std::chrono::duration<double, std::milli>(finish_ts - start_ts).count();

      TestResult result;
      result.yaw_cmd_deg = yaw_step_deg;
      result.pitch_cmd_deg = pitch_step_deg;
      result.delay_ms = delay_ms;
      result.yaw_err_deg = last_error[0];
      result.pitch_err_deg = last_error[1];
      results.push_back(result);

      if (!reached) {
        tools::logger()->warn("[GimbalDelayTest] run {} timeout: yaw_err={:.2f}deg pitch_err={:.2f}deg delay_ms={:.2f}",
          i + 1, last_error[0], last_error[1], delay_ms);
      } else {
        tools::logger()->info("[GimbalDelayTest] run {} reach_ok delay_ms={:.2f} yaw_err={:.2f}deg pitch_err={:.2f}deg",
          i + 1, delay_ms, last_error[0], last_error[1]);
      }

      std::this_thread::sleep_for(500ms);
      send_step(cboard, 0.0, 0.0);
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

    tools::logger()->info("[GimbalDelayTest] CSV saved to {}", log_path);
    return 0;
  } catch (const std::exception & e) {
    tools::logger()->error("[GimbalDelayTest] failed: {}", e.what());
    return 1;
  }
}
