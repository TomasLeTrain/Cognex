#pragma once

#include "apis.h"
//
#include "globals/config.h"
#include "globals/device_globals.h"

/*
 * vexmaps configuration
 * */

using namespace vexmaps;

// pointers to be able to change the pose getter
// TODO: replace all that with model manager
// extern vexmaps::LocalizationModel* pose_getter;
// extern vexmaps::LocalizationModel* orientation_getter;
// extern pros::Mutex pose_mutex;

extern ModelManager model_manager;

// trackers
extern vexmaps::MotorGroupTracking left_dt_tracker;
extern vexmaps::MotorGroupTracking right_dt_tracker;

extern vexmaps::HorizontalOdometryTracker horizontal_tracker;
extern vexmaps::VerticalOdometryTracker vertical_tracker;

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

using laser_model_type =
  vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>;

// clang-format off
extern laser_model_type front_laser_model;
extern laser_model_type left_laser_model;
extern laser_model_type back_laser_model;
extern laser_model_type right_laser_model;
// clang-format on

extern vexmaps::ParticleFilterModel<pf_particle_count> pf_model;

extern vexmaps::SmootherModel smoother_model;
