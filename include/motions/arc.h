#pragma once

#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "motions/utils.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace motions {

struct ArcParams {
    /** whether the robot should turn to face the point with the front of the
     * robot. True by default */
    bool forwards = true;

    /** angle between the robot and target point where the movement will
     * exit. */
    double earlyExitRange = 0;

    uint32_t minSpeed = 0;
    uint32_t maxSpeed = 0;
};

inline void moveArc(Length radius,
                    float angle,
                    LinearVelocity velocity,
                    lemlib::AngularDirection direction,
                    int timeout,
                    ArcParams params = {},
                    bool async = true) {

    // const Angle target_angle = units::constrainAngle360_2(from_cDeg(angle));
    const Length tc_2 = track_width / 2;
    Angle starting_angle = from_cDeg(chassis.getPose().theta);

    // 1 if its moving to the left, else its -1
    float sign =
      direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE ? 1 : -1;

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

    if(!params.forwards){
        // swap l and r, then make them negative
        auto tmp_v_L = v_L;
        v_L = -v_R;
        v_R = -tmp_v_L;
    }

    units::Vector2D<AngularVelocity> unormalized_vels =
      units::Vector2D<AngularVelocity>(linearToMotorRPM(v_L), // vel left
                                       linearToMotorRPM(v_R) // vel right
      );

    units::Vector2D<AngularVelocity> normalized_vels =
      normalizeRPM(unormalized_vels);

    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;

    chassis.customMotion(
      [&](lemlib::Pose pose) mutable -> lemlib::CustomMotionUpdate {
          Angle current_angle = from_cDeg(chassis.getPose().theta);
          bool settled = false;

          float distTraveled = std::abs(lemlib::angleError(to_cDeg(current_angle),
                                             to_cDeg(starting_angle),
                                             false));
          // calculate deltaTheta
          float deltaTheta = lemlib::angleError(angle,
                                        to_cDeg(current_angle),
                                        false);
          if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;
          
          if (lemlib::sgn(deltaTheta) != lemlib::sgn(prevDeltaTheta)) settled = true;

          // close enough to the target
          if (std::abs(deltaTheta) < units::max(params.earlyExitRange, 2)) {
              settled = true;
          }

          return { static_cast<int>(normalized_vels.x.convert(rpm)),
                   static_cast<int>(normalized_vels.y.convert(rpm)),
                   true, // using velocities
                   settled,
                   distTraveled
                   };
      },
      timeout,
      async);
}

inline void moveArc_centerPoint(units::V2Position center_point,
                                     float angle,
                                     LinearVelocity velocity,
                                     lemlib::AngularDirection direction,
                                     int timeout,
                                     ArcParams params = {},
                                     bool async = true) {
    units::V2Position start_position { chassis.getPose().x * in,
                                       chassis.getPose().y * in };
    const Length radius = center_point.distanceTo(start_position);
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
