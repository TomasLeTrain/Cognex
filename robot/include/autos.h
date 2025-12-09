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

void setMaxDistanceThresholdAll(FLength new_length);
void resetMaxDistanceThresholdAll(
  FLength default_distance = distance_sensor_config.maxDistanceDifference);
void setSmootherAlphas(std::optional<float> new_alpha_x,
                       std::optional<float> new_alpha_y);
void resetSmootherConfig(SmootherConfig default_config = smoother_config);

// changes vexmaps tracker whose pose is used
void changePoseGetter(vexmaps::LocalizationModel* new_getter);

// effectively resets to whatever mcl measures
void DistanceSensorReset(int timeout = 150, double new_alpha = 0.8);

// resets using passed in lasers
// orientation should be as close to an axis as possible
void LaserResets(std::vector<vexmaps::DistanceSensorModel*> enabled_lasers,
                 bool x = true,
                 bool y = true);
