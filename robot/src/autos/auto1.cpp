/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add it to "autos.h"
 */

#include "autos.h"

// do not do anything outside here!

namespace auton1 {

    // you can add any variables / functions here

    void run() {
        // do whatever you want here
        RobotSetPose(48_in, -24_in, 0);
        chassis.turnToPoint(47,-47,2000);
        chassis.moveToPoint(47,-47,2000);
        chassis.turnToPoint(70,-47,2000);
        chassis.moveToPoint(54,-47,2000);
    }

} // namespace auton1
