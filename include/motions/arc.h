#pragma once

#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "motions/utils.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace motions {

struct ArcParams {
    /** whether the robot should turn to face the point with the front of the
     * robot. True by default */
    bool forwards = true;

    /** distance between the robot and target point where the movement will
     * exit. */
    Angle earlyExitRange = 0_stDeg;

    uint32_t minSpeed = 0;
    uint32_t maxSpeed = 0;
};

/*
 * Normalizes an angle to be within +/-180 degrees of the current heading.
 * - angle: The target angle to normalize.
 */
double normalizeTarget(double angle) {
    // Adjust angle to be within +/-180 degrees of the inertial sensor's
    // rotation
    if (angle - imu.get_rotation() > 180) {
        while (angle - imu.get_rotation() > 180) angle -= 360;
    } else if (angle - imu.get_rotation() < -180) {
        while (angle - imu.get_rotation() < -180) angle += 360;
    }
    return angle;
}

/*
 * Ensures output values are at least the specified minimum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - min_output: Minimum allowed output voltage.
 */
void scaleToMin(double& left_output, double& right_output, double min_output) {
    // Scale outputs to ensure minimum voltage is met for both sides
    if (fabs(left_output) <= fabs(right_output) && left_output < min_output &&
        left_output > 0) {
        right_output = right_output / left_output * min_output;
        left_output = min_output;
    } else if (fabs(right_output) < fabs(left_output) &&
               right_output < min_output && right_output > 0) {
        left_output = left_output / right_output * min_output;
        right_output = min_output;
    } else if (fabs(left_output) <= fabs(right_output) &&
               left_output > -min_output && left_output < 0) {
        right_output = right_output / left_output * -min_output;
        left_output = -min_output;
    } else if (fabs(right_output) < fabs(left_output) &&
               right_output > -min_output && right_output < 0) {
        left_output = left_output / right_output * -min_output;
        right_output = -min_output;
    }
}

/*
 * Ensures output values do not exceed the specified maximum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - max_output: Maximum allowed output voltage.
 */
void scaleToMax(double& left_output, double& right_output, double max_output) {
    // Scale outputs to ensure maximum voltage is not exceeded for both sides
    if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
        right_output = right_output / left_output * max_output;
        left_output = max_output;
    } else if (fabs(right_output) > fabs(left_output) &&
               right_output > max_output) {
        left_output = left_output / right_output * max_output;
        right_output = max_output;
    } else if (fabs(left_output) > fabs(right_output) &&
               left_output < -max_output) {
        right_output = right_output / left_output * -max_output;
        left_output = -max_output;
    } else if (fabs(right_output) > fabs(left_output) &&
               right_output < -max_output) {
        left_output = left_output / right_output * -max_output;
        right_output = -max_output;
    }
}

inline void moveArc(Length radius,
                    float angle,
                    LinearVelocity velocity,
                    lemlib::AngularDirection direction,
                    int timeout,
                    ArcParams params = {},
                    bool async = true) {

    const Angle target_angle = units::constrainAngle360_2(from_cDeg(angle));
    const Length tc_2 = track_width / 2;

    // Angle starting_angle = from_cDeg(chassis.getPose().theta);

    // 1 if its moving to the left, else its -1
    // float sign =
    //   direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE ? 1 : -1;

    // flips the velocities based on the direction its turning
    // const Length signed_radius = radius * sign;

    // v_l = v_c - w * r_b
    // v_r = v_c + w * r_b
    //
    // v_l = v_c - (v_c / r) * (tc / 2)
    // v_r = v_c + (v_c / r) * (tc / 2)
    //
    // v_l = v_c * (1 - (1 / r) * (tc / 2))
    // v_r = v_c * (1 + (1 / r) * (tc / 2))
    //
    // v_l = v_c * (1 - tc_2 / r)
    // v_r = v_c * (1 + tc_2 / r)

    // LinearVelocity v_L = velocity * (1 - tc_2 / signed_radius);
    // LinearVelocity v_R = velocity * (1 + tc_2 / signed_radius);
    //
    // units::Vector2D<AngularVelocity> unormalized_vels =
    //   units::Vector2D<AngularVelocity>(linearToMotorRPM(v_L), // vel left
    //                                    linearToMotorRPM(v_R) // vel right
    //   );
    //
    // units::Vector2D<AngularVelocity> normalized_vels =
    //   normalizeRPM(unormalized_vels);

    // Normalize the target angle to be within +/-180 degrees of the current
    // heading
    Angle result_angle_deg = from_stDeg(normalizeTarget(angle));
    auto start_angle = from_cDeg(imu.get_rotation());
    //
    Angle angle_to_travel = result_angle_deg - start_angle;

    // Calculate arc lengths for inner and outer wheels
    Multiplied<Length, Angle> in_arc =
      units::abs((units::abs(radius) - tc_2) * angle_to_travel);
    Multiplied<Length, Angle> out_arc =
      units::abs((units::abs(radius) + tc_2) * angle_to_travel);
    double ratio = in_arc / out_arc;

    // stopChassis(vex::brakeType::coast);

    // Determine curve and drive direction
    int curve_direction = radius.internal() > 0 ? 1 : -1;

    int drive_direction = 0;

    // same sign
    if (units::sgn(radius) == units::sgn(angle_to_travel)) {
        drive_direction = 1;
    } else {
        drive_direction = -1;
    }

    // Slew rate and minimum speed logic for chaining
    double max_slew_accel_fwd = 24;
    double max_slew_decel_fwd = 24;
    double max_slew_accel_rev = 24;
    double max_slew_decel_rev = 24;

    double max_slew_fwd =
      drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
    double max_slew_rev =
      drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;

    chassis.lateralPID.reset();
    chassis.angularPID.reset();

    uint32_t start_time = pros::millis();
    double left_output = 0, right_output = 0, correction_output = 0;
    Multiplied<Length, Angle> current_right = 0 * m * rad,
                              current_left = 0 * m * rad;
    Angle current_angle = from_cDeg(imu.get_rotation());

    Length left_start_distance = getDistanceTraveled(drivetrain.leftMotors);
    Length right_start_distance = getDistanceTraveled(drivetrain.rightMotors);

    chassis.customMotion(
      [&](lemlib::Pose pose) mutable -> lemlib::CustomMotionUpdate {
          bool settled = false;
          if (params.minSpeed != 0.0) {
              if (units::abs(target_angle - current_angle) <
                  units::max(params.earlyExitRange, 1_stDeg)) {
                  settled = true;
              }
          } else {
              settled = current_right < out_arc;
          }

          // Main control loop for each curve/exit configuration
          if (!settled) {
              current_angle = from_cDeg(imu.get_rotation());
              Angle real_angle = 0_stDeg;

              if (curve_direction == -1) {
                  // left curve
                  current_right = (getDistanceTraveled(drivetrain.rightMotors) -
                                   right_start_distance) *
                                  rad;

                  // real angle along the arc - basically linear interpolation
                  // between target angle and start_angle
                  Angle right_real_angle =
                    current_right / out_arc * (result_angle_deg - start_angle) +
                    start_angle;

                  real_angle = right_real_angle;

                  auto lateral_error = out_arc - current_right;

                  right_output =
                    chassis.lateralPID.update(lateral_error.internal()) *
                    drive_direction;
                  left_output = right_output * ratio;

              } else if (curve_direction == 1) {
                  // right curve
                  current_left = (getDistanceTraveled(drivetrain.leftMotors) -
                                  left_start_distance) *
                                 rad;
                  Angle left_real_angle =
                    current_left / out_arc * (result_angle_deg - start_angle) +
                    start_angle;
                  real_angle = left_real_angle;

                  auto lateral_error = out_arc - current_left;

                  left_output =
                    chassis.lateralPID.update(lateral_error.internal()) *
                    drive_direction;
                  right_output = left_output * ratio;
              }

              Angle new_target_angle =
                from_stDeg(normalizeTarget(real_angle.internal()));
              Angle angle_error = new_target_angle - current_angle;

              correction_output =
                chassis.lateralPID.update(angle_error.internal());

              // Enforce minimum output if chaining
              if (params.minSpeed != 0) {
                  scaleToMin(left_output, right_output, params.minSpeed);
              }

              // Apply heading correction
              left_output += correction_output;
              right_output -= correction_output;

              // Enforce maximum output
              scaleToMax(left_output, right_output, params.maxSpeed);
          }

          return { static_cast<int>(left_output),
                   static_cast<int>(right_output),
                   false, // using voltage
                   settled,
                   (getDistanceTraveled(drivetrain.rightMotors).internal() +
                    getDistanceTraveled(drivetrain.leftMotors).internal()) /
                     2 };
      },
      timeout,
      async);

    // chassis.customMotion(
    //     [&](lemlib::Pose pose) mutable -> lemlib::CustomMotionUpdate {
    //         Angle current_angle = from_cDeg(chassis.getPose().theta);
    //         bool settled = false;
    //
    //         if(smallestAbsoluteAngleDifference(target_angle, current_angle) <
    //                 units::max(params.earlyExitRange,1_stDeg)){
    //             settled = true;
    //         }
    //
    //         // here radians is technically a dimensionless unit, so we need
    //         to remove its component
    //         // distance left in the motion
    //         // Length d_L = r_L * dtheta / rad;
    //         // Length d_R = r_R * dtheta / rad;
    //         //
    //         // Length d_c = radius * dtheta / rad;
    //         //
    //         // Time t = d_c / velocity;
    //         //
    //         // LinearVelocity v_L = d_L / t;
    //         // LinearVelocity v_R = d_R / t;
    //
    //         return {
    //             static_cast<int>(normalized_vels.x.convert(rpm)),
    //             static_cast<int>(normalized_vels.y.convert(rpm)),
    //             true,  // using velocities
    //             settled,
    //             to_stDeg(current_angle - starting_angle)
    //         };
    //      },
    //     timeout,
    //     async
    // );
}

inline void moveStraight_centerPoint(units::V2Position rotation_point,
                                     float angle,
                                     LinearVelocity velocity,
                                     lemlib::AngularDirection direction,
                                     int timeout,
                                     ArcParams params = {},
                                     bool async = true) {
    units::V2Position start_position { chassis.getPose().x * in,
                                       chassis.getPose().y * in };
    const Length radius = rotation_point.distanceTo(start_position);
    moveArc(radius, angle, velocity, direction, timeout, params, async);
}

// should later figure out the radius required to get to the end point
// inline void moveStraight_EndPoint(
//     units::V2Position end_point,
//     float angle,
//     LinearVelocity velocity,
//     lemlib::AngularDirection direction,
//     int timeout,
//     ArcParams params = {},
//     bool async = true
//     ){
//         units::V2Position start_position {
//         chassis.getPose().x * in,
//         chassis.getPose().y * in
//         };
//
//         const Length radius = rotation_point.distanceTo(start_position);
//         moveArc( radius, angle, velocity, direction, timeout, params, async);
//     }
} // namespace motions
