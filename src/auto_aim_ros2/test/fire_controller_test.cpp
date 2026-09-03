#include "auto_aim_ros2/fire_controller.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace
{

using Controller = auto_aim_ros2::FireController;
using namespace std::chrono_literals;

bool expect(int actual, int expected, const std::string &label)
{
  if (actual == expected)
  {
    return true;
  }
  std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
  return false;
}

}  // namespace

int main()
{
  const auto config = auto_aim_ros2::FireControllerConfig{0.15, 0.5, 0.1, 3};
  const auto start = Controller::TimePoint{1s};
  bool ok = true;

  Controller burst(config);
  ok &= expect(burst.update(true, start), 0, "hold begins low");
  ok &= expect(burst.update(true, start + 149ms), 0, "hold not complete");
  ok &= expect(burst.update(true, start + 150ms), 1, "first pulse");
  ok &= expect(burst.update(true, start + 151ms), 0, "low edge after first pulse");
  ok &= expect(burst.update(true, start + 249ms), 0, "shot period enforced");
  ok &= expect(burst.update(true, start + 250ms), 1, "second pulse");
  ok &= expect(burst.update(true, start + 251ms), 0, "low edge after second pulse");
  ok &= expect(burst.update(true, start + 350ms), 1, "third pulse");
  ok &= expect(burst.update(true, start + 351ms), 0, "low edge after final pulse");
  ok &= expect(burst.update(true, start + 849ms), 0, "between-burst interval");
  ok &= expect(burst.update(true, start + 850ms), 1, "next burst after interval");

  Controller cancelled(config);
  ok &= expect(cancelled.update(true, start), 0, "cancel test hold begins");
  ok &= expect(cancelled.update(true, start + 150ms), 1, "cancel test first pulse");
  ok &= expect(cancelled.update(false, start + 160ms), 0, "loss cancels immediately");
  ok &= expect(cancelled.update(true, start + 300ms), 0, "relock restarts hold");
  ok &= expect(cancelled.update(true, start + 449ms), 0, "relock hold not complete");
  ok &= expect(cancelled.update(true, start + 450ms), 0, "cancelled burst cooldown remains");
  ok &= expect(cancelled.update(true, start + 650ms), 1, "relock fires after hold and cooldown");

  return ok ? 0 : 1;
}
