#ifndef M6A10_V12_TEST_STUB_ROS_H
#define M6A10_V12_TEST_STUB_ROS_H

#include <chrono>
#include <cstdint>

namespace ros {

class WallTime
{
public:
  static WallTime now()
  {
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now().time_since_epoch();
    return WallTime(elapsed.count());
  }

  double toSec() const { return seconds_; }

private:
  explicit WallTime(double seconds) : seconds_(seconds) {}
  double seconds_ = 0.0;
};

class Time
{
public:
  Time() = default;
  explicit Time(std::uint32_t seconds) : seconds_(seconds) {}

  static Time now() { return Time(); }

  Time fromSec(double seconds)
  {
    seconds_ = static_cast<std::uint32_t>(seconds);
    return *this;
  }

private:
  std::uint32_t seconds_ = 0;
};

}  // namespace ros

#endif  // M6A10_V12_TEST_STUB_ROS_H
