#pragma once

#include "apis.h"
//
#include "globals/config.h"
#include "globals/device_globals.h"

/*
 * vexmaps configuration
 * */

using namespace vexmaps;

extern ModelManager model_manager;
extern BlazingWrapper vexmaps_tracker;

// trackers
extern vexmaps::MotorGroupTracking left_dt_tracker;
extern vexmaps::MotorGroupTracking right_dt_tracker;

extern vexmaps::HorizontalOdometryTracker horizontal_tracker;
extern vexmaps::VerticalOdometryTracker vertical_tracker;

// lists of intalled trackers
extern std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers;
extern std::initializer_list<VerticalOdometryTracker*> vertical_trackers;

extern vexmaps::PfMotionModel<vexmaps::OdometryModel> pf_motion_model;

// distance sensors
extern units::Pose front_distance_offsets;
extern units::Pose left_distance_offsets;
extern units::Pose back_distance_offsets;
extern units::Pose right_distance_offsets;

// clang-format off
extern vexmaps::DistanceSensorModel front_laser_model;
extern vexmaps::DistanceSensorModel left_laser_model;
extern vexmaps::DistanceSensorModel back_laser_model;
extern vexmaps::DistanceSensorModel right_laser_model;
// clang-format on

extern vexmaps::ParticleFilterModel<pf_particle_count> pf_model;

extern vexmaps::SmootherModel smoother_model;
