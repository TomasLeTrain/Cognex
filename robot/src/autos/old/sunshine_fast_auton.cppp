#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Vector2D.hpp"
#include "vexmaps/mcl/distance_model.hpp"

// do not do anything outside here!

namespace sunshine_fast_auton {

// you can add any variables / functions here

void run_auton() {
    RobotSetPose({ 49_in, -16.8_in, 200 * deg });
    intake::setColorSortEnabled(false);

    // TODO: make distance autmatic
    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto difference = (target - RobotGetPose()).normalize() *
                          (RobotGetPose().distanceTo(target) - distance);
        ;
        auto new_point = RobotGetPose() + difference;

        return new_point;
    };

    units::V2FPosition target_point;

    intake::set(intake::intake);

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match = normal_match;

    mb.moveTo(24_in, -24_in)
        .drive_errorTolerance(5_in)
        // .drive_decelSlew(0 * volt)
        .drive_toleranceDuration(0_msec) |
      run;

    mb.moveTo(48_in, -match)
        // .drive_errorTolerance(5_in)
        // .drive_decelSlew(0 * volt)
        .drive_toleranceDuration(0_msec) |
      run;

    mb.turnTo(70_in, -match)
        // .drive_errorTolerance(5_in)
        .turn_toleranceDuration(0_msec) |
      run;

    mb.moveTo(58_in, -match)
        // .drive_errorTolerance(5_in)
        .drive_toleranceDuration(0_msec) |
      run;

    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;
}

} // namespace sunshine_fast_auton
