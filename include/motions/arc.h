#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "globals.h"
#include "units/Angle.hpp"
#include "units/units.hpp"

#include "motions/utils.h"

#include "units/Vector2D.hpp"

#include "globals.h"

namespace motions {

    struct ArcParams {
        /** whether the robot should turn to face the point with the front of the robot. True by default */
        bool forwards = true;

        /** distance between the robot and target point where the movement will exit. */
        Angle earlyExitRange = 0_stDeg;
    };

    void moveArc(
        Length radius,
        float angle,
        LinearVelocity velocity,
        lemlib::AngularDirection direction,
        int timeout,
        ArcParams params = {},
        bool async = true
        ){

        const Angle target_angle = units::constrainAngle360_2(from_cDeg(angle));
        const Length tc_2 = track_width / 2;

        Angle starting_angle = from_cDeg(chassis.getPose().theta);

        // 1 if its moving to the left, else its -1
        float sign = direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE ? 1 : -1;
 
        // flips the velocities based on the direction its turning
        const Length signed_radius = radius * sign;

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

        LinearVelocity v_L = velocity * (1 - tc_2 / signed_radius);
        LinearVelocity v_R = velocity * (1 + tc_2 / signed_radius);

        units::Vector2D<AngularVelocity> unormalized_vels = units::Vector2D<AngularVelocity>(
            linearToMotorRPM(v_L), // vel left
            linearToMotorRPM(v_R)  // vel right
        );

        units::Vector2D<AngularVelocity> normalized_vels = normalizeRPM(unormalized_vels);

        chassis.customMotion(
            [&](lemlib::Pose pose) mutable -> lemlib::CustomMotionUpdate {
                Angle current_angle = from_cDeg(chassis.getPose().theta);
                bool settled = false;

                if(smallestAbsoluteAngleDifference(target_angle, current_angle) <
                        units::max(params.earlyExitRange,1_stDeg)){
                    settled = true;
                }

                // here radians is technically a dimensionless unit, so we need to remove its component
                // distance left in the motion
                // Length d_L = r_L * dtheta / rad;
                // Length d_R = r_R * dtheta / rad;
                //
                // Length d_c = radius * dtheta / rad;
                //
                // Time t = d_c / velocity;
                //
                // LinearVelocity v_L = d_L / t;
                // LinearVelocity v_R = d_R / t;

                return {
                    static_cast<int>(normalized_vels.x.convert(rpm)),
                    static_cast<int>(normalized_vels.y.convert(rpm)),
                    true,  // using velocities
                    settled,
                    to_stDeg(current_angle - starting_angle)
                };
             },
            timeout,
            async
        );
    }

    void moveStraight_centerPoint(
        units::V2Position rotation_point,
        float angle,
        LinearVelocity velocity,
        lemlib::AngularDirection direction,
        int timeout,
        ArcParams params = {},
        bool async = true
        ){
            units::V2Position start_position {
            chassis.getPose().x * in,
            chassis.getPose().y * in
            };
            const Length radius = rotation_point.distanceTo(start_position);           
            arc( radius, angle, velocity, direction, timeout, params, async);
        }

    void moveStraight_EndPoint(
        units::V2Position rotation_point,
        units::V2Position end_point,
        float angle,
        LinearVelocity velocity,
        lemlib::AngularDirection direction,
        int timeout,
        ArcParams params = {},
        bool async = true
        ){
            units::V2Position start_position {
            chassis.getPose().x * in,
            chassis.getPose().y * in
            };

            const Length radius = rotation_point.distanceTo(start_position);           
            arc( radius, angle, velocity, direction, timeout, params, async);
        }
}

