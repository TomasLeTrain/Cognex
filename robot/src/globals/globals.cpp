#include "globals.h"

// here goes configs which almost never change
// but its still benefitial for compile times to have here

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Mutex pose_mutex;

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

