#pragma once

#include "globals.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace motions {
static const Length track_width = drivetrain.trackWidth * in;
static const Length wheel_diameter = drivetrain.wheelDiameter * in;
// angular in / angular out
static const float gear_ratio = 600 / drivetrain.rpm;
static const float dt_rpm = drivetrain.rpm;

/**
 * @brief Returns the motor RPM required for the robot to travel at some linear
 * velocity. Takes wheel diameter and gear ratio into account.
 *
 * @param vel target velocity of the robot
 * @return rpm of the motors to reach that target velocity
 */
static AngularVelocity linearToMotorRPM(LinearVelocity vel) {
    // w = v / r
    // angular vel = vel / (wheel diameter / 2)
    // angular vel = (2 * vel) / wheel diameter
    return (((2 * vel) / wheel_diameter
             //  need to convert based on gear ratio
             ) *
            gear_ratio
            // already returns in right units except for the angular component
            ) *
           rad;
}

/**
 * @brief Returns the motor RPM required for the robot to travel at some linear
 * velocity. Takes wheel diameter and gear ratio into account.
 *
 * @param ang_vel RPM of the motors
 * @return linear velocity of the robot
 */
static LinearVelocity motorRPMToLinearVel(AngularVelocity ang_vel) {
    // v = w * r
    // vel = angular vel * (wheel diameter / 2)
    return ((ang_vel * wheel_diameter / 2)
            //  need to convert based on gear ratio
            / gear_ratio
            // already returns in right units except for the angular component
            ) /
           rad;
}

/**
 * @brief converts angular velocity from wheels into linear velocity. Does not
 * take gear ratio into account
 *
 * @param ang_vel angular velocity of the wheels
 * @return linear velocity of the robot
 */
static LinearVelocity angularToLinearVel(AngularVelocity ang_vel) {
    // v = w * r
    return (ang_vel * wheel_diameter / 2) / rad;
}

/**
 * @brief converts linear velocity of the robot to angular velocity of the
 * wheels. Does not take gear ratio into account.
 *
 * @param lin_vel linear velocity of the robot
 * @return angular velocity of the wheels
 */
static AngularVelocity linearToAngularVel(LinearVelocity lin_vel) {
    // v = w * r
    return ((2 * lin_vel) / wheel_diameter) * rad;
}

/**
 * @brief Reduces motor RPM while keeping the left/right drivetrain ratio
 *
 * @param vector vector of left and right RPM's
 * @return Reduced velocities
 */
inline units::Vector2D<AngularVelocity>
normalizeRPM(units::Vector2D<AngularVelocity> vector) {
    float larger_magnitude = (units::max(vector.x, vector.y) / dt_rpm).internal();

    if (larger_magnitude > 1.0) {
        return { vector.y / larger_magnitude, vector.x / larger_magnitude };
    }
    return vector;
}

static Angle smallestAbsoluteAngleDifference(Angle a1, Angle a2) {
    // here we assume angle and last_angle are both postive and below 360

    // makes sure differences in angles are kept counter-clockwise
    //
    // here it is guarateed that angle > last_angle
    const Angle bigger_angle = units::max(a1, a2);
    const Angle smaller_angle = units::min(a1, a2);

    Angle difference = bigger_angle - smaller_angle;

    //
    // we always assume we took the shortest path to the current angle
    // if the difference is greater than 180 then going the other way is faster
    if (difference > rot / 2) {
        difference = rot - difference;
    }
    return difference;
}

static Angle AngleError(Angle current_angle, Angle last_angle) {
    // here we assume angle and last_angle are both postive and below 360

    // makes sure differences in angles are kept counter-clockwise
    //
    // here it is guarateed that angle > last_angle

    // if(current_angle < last_angle){
    //     // current_angle = 0
    //     // last_angle = 358
    //     //
    //     // actual_change = 2
    //     // change_given = -358
    //
    //     // 0
    // -2
    // }

    Angle difference = units::constrainAngle180(current_angle) -
                       units::constrainAngle180(last_angle);
    return difference;
}
} // namespace motions
