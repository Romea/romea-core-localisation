// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFUpdaterLeaderPose.hpp"

namespace romea
{

// TODO(jean) add update stage
//-----------------------------------------------------------------------------
R2RLocalisationPFUpdaterLeaderPose::R2RLocalisationPFUpdaterLeaderPose(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & /*numberOfParticles*/,
  const double & /*maximalMahalanobisDistance*/,
  const std::string & logFilename)
: LocalisationUpdaterExteroceptive(updaterName,
    minimalRate,
    triggerMode,
    logFilename)
{
}


//--------------------------------------------------------------------------
void R2RLocalisationPFUpdaterLeaderPose::update(
  const Duration & duration,
  const Observation & currentObservation,
  LocalisationFSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rateDiagnostic_.evaluate(duration);

  switch (currentFSMState) {
    case LocalisationFSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        std::cout << " FSM : INIT DONE, GO TO RUNNING MODE " << std::endl;
        currentFSMState = LocalisationFSMState::RUNNING;
      }
      break;
    default:
      break;
  }
}


//-----------------------------------------------------------------------------
bool R2RLocalisationPFUpdaterLeaderPose::set_(
  const Duration & duration,
  const Observation & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddOn)
{
  if (!std::isnan(currentInput.U(R2RLocalisationPFMetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(R2RLocalisationPFMetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY)))
  {
    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(currentObservation.Y(), currentObservation.R());
    randomGenerator.fill(currentState.particles);

    currentAddOn.lastExteroceptiveUpdate.time = duration;
    currentAddOn.lastExteroceptiveUpdate.travelledDistance = currentAddOn.travelledDistance;

    assert(!std::isnan(currentState.particles(0, 0)));
    assert(!std::isnan(currentState.particles(0, 1)));
    assert(!std::isnan(currentState.particles(0, 2)));

    return true;
  } else {
    return false;
  }
}

}  // namespace romea
