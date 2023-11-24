// romea
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterPosition.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
R2WLocalisationPFUpdaterPosition::R2WLocalisationPFUpdaterPosition(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & numberOfParticles,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: LocalisationUpdaterExteroceptive(updaterName,
    minimalRate,
    triggerMode,
    logFilename),
  PFGaussianUpdaterCore(numberOfParticles,
    maximalMahalanobisDistance),
  levelArms_(RowMajorMatrix::Zero(2, numberOfParticles)),
  cosCourses_(RowMajorVector::Zero(numberOfParticles)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles)),
  levelArmCompensation_()
{
}

//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPosition::update(
  const Duration & duration,
  const Observation & currentObservation,
  LocalisationFSMState & currentFSMState,
  MetaState & currentMetaState)
{
  switch (currentFSMState) {
    case LocalisationFSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        std::cout << " FSM : INIT DONE (POSITION + COURSE), GO TO RUNNING MODE " << std::endl;
        currentFSMState = LocalisationFSMState::RUNNING;
      }
      break;
    case LocalisationFSMState::RUNNING:
      if (triggerMode_ == TriggerMode::ALWAYS) {
        try {
          update_(
            duration,
            currentObservation,
            currentMetaState.state,
            currentMetaState.addon);
        } catch (...) {
          std::cout << " FSM : FILTER DEGENERECENCE, RESET AND GO TO INIT MODE" << std::endl;
          currentFSMState = LocalisationFSMState::INIT;
          currentMetaState.state.reset();
          currentMetaState.addon.reset();
        }
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPosition::update_(
  const Duration & duration,
  Observation currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  // compute antenna attitude compensation
  levelArmCompensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.rollPitchVariance,
    0,
    0,
    currentObservation.levelArm);


  double varxyantenna = levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0).trace();
  const Eigen::Vector3d & antennaPosition = levelArmCompensation_.getPosition();
  const double & xantenna = antennaPosition(0);
  const double & yantenna = antennaPosition(1);

  // compute apriori observations
  const auto & courses = currentState.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = currentState.particles.row(MetaState::POSITION_X);
  const auto & y = currentState.particles.row(MetaState::POSITION_Y);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();
  aprioriObservations_.row(MetaState::POSITION_X) = x +
    (cosCourses_ * xantenna - sinCourses_ * yantenna);
  aprioriObservations_.row(MetaState::POSITION_Y) = y +
    (sinCourses_ * xantenna + cosCourses_ * yantenna);

  // update weights and resample
  currentObservation.R(MetaState::POSITION_X, MetaState::POSITION_X) += varxyantenna;
  currentObservation.R(MetaState::POSITION_Y, MetaState::POSITION_Y) += varxyantenna;
  if (updateState_(currentState, currentObservation)) {
    currentAddon.lastExteroceptiveUpdate.time = duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
  }
}

//-----------------------------------------------------------------------------
bool R2WLocalisationPFUpdaterPosition::set_(
  const Duration & duration,
  Observation const & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddon)
{
  currentAddon.lastExteroceptiveUpdate.time = duration;
  currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;

  if (!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentState.particles(MetaState::ORIENTATION_Z, 0)))
  {
    computeLevelArms_(currentObservation, currentAddon);
    setParticlePositions_(currentObservation, currentState);
    applyLevelArmCompentations_(currentState);
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPosition::computeLevelArms_(
  Observation const & currentObservation,
  const AddOn & currentAddon)
{
  NormalRandomArrayGenerator2D<double> randomGenerator;

  levelArmCompensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.rollPitchVariance,
    0,
    0,
    currentObservation.levelArm);

  randomGenerator.init(
    levelArmCompensation_.getPosition().segment<2>(0),
    levelArmCompensation_.getPosition().block<2, 2>(0, 0));

  randomGenerator.fill(levelArms_);
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPosition::setParticlePositions_(
  const Observation & currentObservation,
  State & currentState)
{
  auto particlePositions = currentState.particles.block(2, numberOfParticles_, 0, 0);

  NormalRandomArrayGenerator2D<double> randomGenerator;
  randomGenerator.init(currentObservation.Y(), currentObservation.R());
  randomGenerator.fill(particlePositions);
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPosition::applyLevelArmCompentations_(State & currentState)
{
  cosCourses_ = currentState.particles.row(MetaState::ORIENTATION_Z).cos();
  sinCourses_ = currentState.particles.row(MetaState::ORIENTATION_Z).sin();

  currentState.particles.row(MetaState::POSITION_X) -=
    cosCourses_ * levelArms_.row(MetaState::POSITION_X) -
    sinCourses_ * levelArms_.row(MetaState::POSITION_Y);

  currentState.particles.row(MetaState::POSITION_Y) -=
    sinCourses_ * levelArms_.row(MetaState::POSITION_X) +
    cosCourses_ * levelArms_.row(MetaState::POSITION_Y);
}

}  // namespace core
}  // namespace romea
