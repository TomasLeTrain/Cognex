#include "blazing/api.hpp"
// #include "vexmaps/api.hpp"

// saves about 10k binary size??
template class blazing::MotionBuilder<blazing::Chassis<blazing::DifferentialDrivetrain, blazing::ArcOdomTracker, blazing::normalLargeChainTolerances<blazing::Tolerances<blazing::ErrorTolerance<Length>, blazing::VelocityTolerance<Length>>, blazing::Tolerances<blazing::ErrorTolerance<Angle>, blazing::VelocityTolerance<Angle>>, blazing::Tolerances<blazing::ErrorTolerance<Length>, blazing::VelocityTolerance<Length>>, blazing::Tolerances<blazing::ErrorTolerance<Angle>, blazing::VelocityTolerance<Angle>>, blazing::Tolerances<blazing::ErrorTolerance<Length>>, blazing::Tolerances<blazing::ErrorTolerance<Angle>>>>, blazing::Controllers<blazing::LinearFeedbackController<blazing::PID<Length, Voltage>>, blazing::AngularFeedbackController<blazing::PID<Angle, Voltage>>, blazing::LinearSlewController, blazing::AngularSlewController, blazing::LinearVoltageClampController, blazing::AngularVoltageClampController>>;
//
// template class vexmaps::PfMotionModel<vexmaps::OdometryModel>;
//
// template class vexmaps::ParticleFilterModel<500>;
