/**
 * @file ranger_mini_v3_base.hpp
 * @brief Motion command encoding for the Ranger Mini 3.0.
 *
 * The Ranger Mini 3.0 firmware reads the motion command frame (0x111) the
 * way the Ranger Mini 1.0 did, not as the protocol table in its manual
 * (V1.0.0, 2024.06) describes:
 *   - bytes 6-7, steering angle: units of 0.01 deg (the table says 0.001 rad;
 *     the manual's own sample frame, "turns 10 deg" = 0x03E8, agrees with
 *     0.01 deg)
 *   - bytes 4-5, spin speed: wheel speed in mm/s (bytes 2-3, which the table
 *     names as spin speed in 0.001 rad/s, are ignored in spin mode)
 * Both were confirmed on a robot by sending raw frames: a steer value of 1000
 * gave ~10 deg, and 300 in bytes 4-5 gave a full spin in ~6 s.
 *
 * ugv_sdk's RangerMiniV3Base sends the standard layout, so this class
 * converts before handing the values to the SDK (which scales every field
 * by 1000).
 */

#ifndef RANGER_MINI_V3_BASE_HPP
#define RANGER_MINI_V3_BASE_HPP

#include <cmath>

#include "ugv_sdk/details/robot_base/ranger_base.hpp"

namespace westonrobot {
class RangerMiniV3CmdBase : public RangerBase {
 public:
  struct EncodedCommand {
    double linear;    // m/s, sent as mm/s in bytes 0-1
    double angular;   // bytes 2-3, unused by the firmware
    double lateral;   // spin wheel speed in m/s, sent as mm/s in bytes 4-5
    double steering;  // degrees / 10, sent as 0.01 deg in bytes 6-7
  };

  // spin_radius: distance from the robot centre to a wheel, in metres
  explicit RangerMiniV3CmdBase(double spin_radius)
      : RangerBase(), spin_radius_(spin_radius) {}

  // steer_angle: inner wheel angle in rad, left positive
  // angular_vel: spin rate in rad/s, counter-clockwise positive
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double angular_vel = 0.0) override {
    auto cmd = Encode(linear_vel, steer_angle, angular_vel, spin_radius_);
    SendMotionCommand(cmd.linear, cmd.angular, cmd.lateral, cmd.steering);
  }

  static EncodedCommand Encode(double linear_vel, double steer_angle,
                               double angular_vel, double spin_radius) {
    EncodedCommand cmd;
    cmd.linear = linear_vel;
    cmd.angular = 0.0;
    cmd.lateral = kSpinSign * angular_vel * spin_radius;
    cmd.steering = steer_angle * 180.0 / M_PI / 10.0;
    return cmd;
  }

  // Sign of bytes 4-5 relative to ROS (counter-clockwise positive). Matches
  // ugv_sdk's Ranger Mini 1.0 handling.
  static constexpr double kSpinSign = -1.0;

 private:
  double spin_radius_;
};
}  // namespace westonrobot

#endif /* RANGER_MINI_V3_BASE_HPP */
