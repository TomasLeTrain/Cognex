#pragma once

#include "globals.h"
#include "pros/motor_group.hpp"
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
inline AngularVelocity linearToMotorRPM(LinearVelocity vel) {
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
inline LinearVelocity motorRPMToLinearVel(AngularVelocity ang_vel) {
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
inline LinearVelocity angularToLinearVel(AngularVelocity ang_vel) {
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
inline AngularVelocity linearToAngularVel(LinearVelocity lin_vel) {
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

inline Angle smallestAbsoluteAngleDifference(Angle a1, Angle a2) {
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

inline Angle AngleError(Angle current_angle, Angle last_angle) {
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

    inline double getGearingTicks(pros::MotorGears gearing) {
        double gearing_multiplier = 1;
        switch (gearing) {
            case pros::MotorGears::blue: gearing_multiplier = 300.0; break;
            case pros::MotorGears::green: gearing_multiplier = 900.0; break;
            case pros::MotorGears::red: gearing_multiplier = 1800.0; break;
            default: gearing_multiplier = 1; break;
        }
        return gearing_multiplier;
    }

    inline double getGearingRPM(pros::MotorGears gearing) {
        double gearing_multiplier = 1;
        switch (gearing) {
            case pros::MotorGears::blue: gearing_multiplier = 600.0; break;
            case pros::MotorGears::green: gearing_multiplier = 200.0; break;
            case pros::MotorGears::red: gearing_multiplier = 100.0; break;
            default: gearing_multiplier = 200.0; break;
        }
        return gearing_multiplier;
    }

    inline Length getDistanceTraveled(pros::MotorGroup* motors) {
        Length distance = 0.0_m;

        if (motors == nullptr) {
            printf("odometry: motor group is a nullptr!\n");
            return 0.0_m;
        }

        double used_motor_count = 0;

        for (int i = 0; i < motors->size(); i++) {
            int port = abs(motors->get_port(i));
            // check if is installed
            auto plugged_device_type =
              (pros::DeviceType)pros::c::registry_get_plugged_type(port - 1);

            // only include if plugged in
            if (plugged_device_type == pros::DeviceType::motor) {
                used_motor_count += 1.0;

                pros::MotorGears gearing = motors->get_gearing(i);
                pros::MotorUnits encoder = motors->get_encoder_units(i);

                double rotation_multiplier =
                  1; // should convert position to # of rotations

                switch (encoder) {
                    case pros::MotorUnits::degrees:
                        rotation_multiplier = 1 / 360.0;
                        break;
                    case pros::MotorUnits::counts:
                        rotation_multiplier = 1 / getGearingTicks(gearing);
                        break;
                    case pros::MotorUnits::rotations:
                        rotation_multiplier = 1;
                        break;
                    default: rotation_multiplier = 1; break;
                }

                double position = motors->get_position(i);

                double gear_ratio = getGearingRPM(gearing) / dt_rpm;

                distance += (position * rotation_multiplier) *
                            (wheel_diameter * M_PI) / gear_ratio;
            }
        }

        if (used_motor_count != 0.0) distance /= used_motor_count;
        return distance;
    }

} // namespace motions
