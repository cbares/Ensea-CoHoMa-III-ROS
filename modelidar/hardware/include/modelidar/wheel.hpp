#ifndef MODELIDAR__WHEEL_HPP_
#define MODELIDAR__WHEEL_HPP_

#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace modelidar
{

/// One wheel: interface names and encoder ticks → radians conversion.
class Wheel
{
public:
  std::string name;
  std::string velocity_name;
  std::string position_name;

  void setup(const std::string & joint_name, int enc_counts_per_rev)
  {
    name = joint_name;
    rad_per_count_ = 2.0 * M_PI / enc_counts_per_rev;
    velocity_name = name + "/" + hardware_interface::HW_IF_VELOCITY;
    position_name = name + "/" + hardware_interface::HW_IF_POSITION;
  }

  /// Makes the current firmware position the origin (on activation).
  void zero(long ticks)
  {
    offset_ = -ticks;
    last_ticks_ = ticks;
  }

  /// The firmware restarted and counts from 0 again: keep the position continuous.
  void firmware_restarted()
  {
    offset_ += last_ticks_;
    last_ticks_ = 0;
  }

  /// Angle (rad) from the firmware position (ticks).
  double position(long ticks)
  {
    last_ticks_ = ticks;
    return static_cast<double>(ticks + offset_) * rad_per_count_;
  }

  /// Angular velocity (rad/s) from the firmware velocity (ticks/s).
  double velocity(double ticks_per_s) const {return ticks_per_s * rad_per_count_;}

private:
  double rad_per_count_ = 0.0;
  long offset_ = 0;
  long last_ticks_ = 0;
};

}  // namespace modelidar

#endif  // MODELIDAR__WHEEL_HPP_
