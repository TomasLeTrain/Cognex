#pragma once

#include "apis.h"
//

// defines all the autons
#include "autons_list.h"

// helpful includes for autons
#include "globals.h"
#include "units/Angle.hpp"

/* auton utils - leave alone */

// updates poses of vexmaps and blazing trackers
void RobotSetPose(units::Pose pose);
void RobotSetPose(double x, double y, double angle);

// gets pose from vexmaps tracker
units::Pose RobotGetPose();

// changes vexmaps tracker whose pose is used
void changePoseGetter(vexmaps::LocalizationModel* new_getter);

// effectively resets to whatever mcl measures
void DistanceSensorReset(int timeout = 150, double new_alpha = 0.8);

// resets using passed in lasers
// orientation should be as close to an axis as possible
void LaserResets(std::vector<laser_model_type*> enabled_lasers,
                 bool x = true,
                 bool y = true);
