#include "globals.h"

vexmaps::LocalizationModel* orientation_getter = nullptr;

vexmaps::ModelManager model_manager(
  {
    { &pf_motion_model, "odom model",     1 },
    { &pf_model,        "pf model",       2 },
    { &smoother_model,  "smoother model", 3 },
},
  &smoother_model);

// if the tracker is not installed the list can be left empty -> tracker = {};
std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers = {
    &horizontal_tracker
};
std::initializer_list<VerticalOdometryTracker*> vertical_trackers = {
    &vertical_tracker
};

// trackers
vexmaps::MotorGroupTracking left_dt_tracker(&left_motors,
                                            drivetrain_config.wheel_diameter,
                                            drivetrain_config.rpm,
                                            -drivetrain_config.track_width / 2);

vexmaps::MotorGroupTracking right_dt_tracker(&right_motors,
                                             drivetrain_config.wheel_diameter,
                                             drivetrain_config.rpm,
                                             drivetrain_config.track_width / 2);

vexmaps::HorizontalOdometryTracker
  horizontal_tracker(&sideways_odom_rotation,
                     sideways_tracker_config.diameter,
                     1,
                     sideways_tracker_config.offset);
vexmaps::VerticalOdometryTracker
  vertical_tracker(&forwards_odom_rotation,
                   forwards_tracker_config.diameter,
                   1,
                   forwards_tracker_config.offset);

// never really changes
vexmaps::PfMotionModel<vexmaps::OdometryModel>
  pf_motion_model(motion_model_config,
                  &left_dt_tracker,
                  &right_dt_tracker,
                  horizontal_trackers,
                  vertical_trackers,
                  &imu,
                  false); // use drivetrain -
                          // can be left on false since it falls back to
                          // drivetrain of no rotations are connected


// clang-format off
laser_model_type front_laser_model(&front_distance, front_distance_offsets, "front");
laser_model_type left_laser_model(&left_distance,   left_distance_offsets,  "left");
laser_model_type back_laser_model(&back_distance,   back_distance_offsets,  "back");
laser_model_type right_laser_model(&right_distance, right_distance_offsets, "right");
// clang-format on

vexmaps::ParticleFilterModel<pf_particle_count> pf_model(&pf_motion_model,
                                                         { // distance sensors
                                                           &front_laser_model,
                                                           &left_laser_model,
                                                           &back_laser_model,
                                                           &right_laser_model },
                                                         Pfconfig);

vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);

