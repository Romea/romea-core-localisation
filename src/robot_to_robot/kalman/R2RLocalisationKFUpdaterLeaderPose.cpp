#include "romea_localisation/robot_to_robot/kalman/R2RLocalisationKFUpdaterLeaderPose.hpp"

#include <iostream>

namespace romea
{

//TODO add update stage
//-----------------------------------------------------------------------------
R2RLocalisationKFUpdaterLeaderPose::R2RLocalisationKFUpdaterLeaderPose(const std::string &updaterName,
                                                                       const double &minimalRate,
                                                                       const TriggerMode &triggerMode,
                                                                       const double &maximalMahalanobisDistance,
                                                                       const std::string &logFilename):
  LocalisationUpdaterExteroceptive(updaterName,
                                   minimalRate,
                                   triggerMode,
                                   logFilename)
{

}

//--------------------------------------------------------------------------
void R2RLocalisationKFUpdaterLeaderPose::update(const Duration &duration,
                                                const Observation &currentObservation,
                                                LocalisationFSMState & currentFSMState,
                                                MetaState &currentMetaState)
{

  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    if(set_(duration,
            currentObservation,
            currentMetaState.input,
            currentMetaState.state,
            currentMetaState.addon))
    {
      std::cout << " FSM : INIT DONE, GO TO RUNNING MODE "<< std::endl;
      currentFSMState = LocalisationFSMState::RUNNING;
    }
    break;
  default:
    break;
  }
}

//-----------------------------------------------------------------------------
bool R2RLocalisationKFUpdaterLeaderPose::set_(const Duration & duration,
                                              const Observation & currentObservation,
                                              const Input &currentInput,
                                              State &currentState,
                                              AddOn &currentAddOn)
{

  currentAddOn.lastExteroceptiveUpdate.time=duration;
  currentAddOn.lastExteroceptiveUpdate.travelledDistance=currentAddOn.travelledDistance;

  if(!std::isnan(currentInput.U(R2RLocalisationKFMetaState::LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationKFMetaState::LINEAR_SPEED_Y_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationKFMetaState::ANGULAR_SPEED_Z_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationKFMetaState::LEADER_LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationKFMetaState::LEADER_LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationKFMetaState::LEADER_LINEAR_SPEED_X_BODY)))
  {
    currentState.X() = currentObservation.Y();
    currentState.P() = currentObservation.R();
    return true;
  }
  else
  {
    return false;
  }
}


}
