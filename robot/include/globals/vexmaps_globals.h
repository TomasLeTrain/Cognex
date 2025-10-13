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
extern vexmaps::LocalizationModel* pose_getter;
extern vexmaps::LocalizationModel* orientation_getter;
extern pros::Mutex pose_mutex;

extern ModelManager model_manager;

// trackers
inline vexmaps::MotorGroupTracking
  left_dt_tracker(&left_motors,
                  drivetrain_config.wheel_diameter,
                  drivetrain_config.rpm,
                  -drivetrain_config.track_width / 2);

inline vexmaps::MotorGroupTracking
  right_dt_tracker(&right_motors,
                   drivetrain_config.wheel_diameter,
                   drivetrain_config.rpm,
                   drivetrain_config.track_width / 2);

extern vexmaps::HorizontalOdometryTracker horizontal_tracker;
extern vexmaps::VerticalOdometryTracker vertical_tracker;

inline vexmaps::HorizontalOdometryTracker
  horizontal_tracker(&sideways_odom_rotation,
                     sideways_tracker_config.diameter,
                     1,
                     sideways_tracker_config.offset);
inline vexmaps::VerticalOdometryTracker
  vertical_tracker(&forwards_odom_rotation,
                   forwards_tracker_config.diameter,
                   1,
                   forwards_tracker_config.offset);

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
inline laser_model_type front_laser_model(&front_distance, front_distance_offsets, "front");
inline laser_model_type left_laser_model(&left_distance,   left_distance_offsets,  "left");
inline laser_model_type back_laser_model(&back_distance,   back_distance_offsets,  "back");
inline laser_model_type right_laser_model(&right_distance, right_distance_offsets, "right");
// clang-format on

inline vexmaps::ParticleFilterModel<pf_particle_count>
  pf_model(&pf_motion_model,
           { // distance sensors
             &front_laser_model,
             &left_laser_model,
             &back_laser_model,
             &right_laser_model },
           Pfconfig);

inline vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);

