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

/**
 * @brief creates function which returns true when robot is within threshold
 * distance to target. Can be used for async/chain motions:
 *
 *
 * @param target target position
 * @param threshold distance between the robot's position and target within
 * which it returns true.
 *
 * @b Example
 * @code {.cpp}
 * // queue up motions
 * mb.turnTo(0, 0) | async;
 * mb.moveTo(0, 0) | async;
 *
 * // wait until robot is within 4 inches of point (-24, 24)
 * async.waitUntil(closeEnough({ -24_in, 24_in }, 4_in));
 * // do something when close
 * intake.outtake();
 * // wait for all motions to finish
 * async.wait();
 * @endcode
 */
std::function<bool()> closeEnough(units::V2Position target, Length threshold);

/**
 * @brief creates function which returns true when robot is within threshold
 * distance to target. Can be used for async/chain motions:
 *
 *
 * @param target_x x coordinate of target position, in inches
 * @param target_y y coordinate of target position, in inches
 * @param threshold distance between the robot's position and target within
 * which it returns true, in inches
 *
 * @b Example
 * @code {.cpp}
 * // queue up motions
 * mb.turnTo(0, 0) | async;
 * mb.moveTo(0, 0) | async;
 *
 * // wait until robot is within 4 inches of point (-24, 24)
 * async.waitUntil(closeEnough(-24, 24, 4));
 * // do something when close
 * intake.outtake();
 * // wait for all motions to finish
 * async.wait();
 * @endcode
 */
std::function<bool()>
closeEnough(double target_x, double target_y, double threshold);

std::shared_ptr<lyfast::geometry::Line>
line(float x0, float y0, float x1, float y1);

std::shared_ptr<lyfast::geometry::CubicBezier> curve(float x0,
                                                     float y0,
                                                     float x1,
                                                     float y1,
                                                     float x2,
                                                     float y2,
                                                     float x3,
                                                     float y3);
