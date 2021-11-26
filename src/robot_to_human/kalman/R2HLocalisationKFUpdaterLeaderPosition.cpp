#include "romea_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterLeaderPosition.hpp"
#include <iostream>

namespace romea
{

//TODO add update stage
//-----------------------------------------------------------------------------
R2HLocalisationKFUpdaterLeaderPosition::R2HLocalisationKFUpdaterLeaderPosition(const std::string &updaterName,
                                                                               const double &minimalRate,
                                                                               const TriggerMode &triggerMode, const double &maximalMahalanobisDistance,
                                                                               const std::string &logFilename):
  LocalisationUpdaterExteroceptive(updaterName,
                                   minimalRate,
                                   triggerMode,
                                   logFilename)
{

}

//--------------------------------------------------------------------------
void R2HLocalisationKFUpdaterLeaderPosition::update(const Duration &duration,
                                                    const Observation &currentObservation,
                                                    LocalisationFSMState & currentFSMState,
                                                    MetaState &currentMetaState)
{
  rateDiagnostic_.evaluate(duration);

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
bool R2HLocalisationKFUpdaterLeaderPosition::set_(const Duration & duration,
                                                  const Observation & currentObservation,
                                                  const Input & currentInput,
                                                  State & currentState,
                                                  AddOn & currentAddOn)
{
  currentAddOn.lastExteroceptiveUpdate.time=duration;
  currentAddOn.lastExteroceptiveUpdate.travelledDistance=currentAddOn.travelledDistance;

  if(!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY))&&
     !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)))
  {
    currentState.X().segment<2>(MetaState::LEADER_POSITION_X)=currentObservation.Y();

    currentState.P().block<2,2>(MetaState::LEADER_POSITION_X,
                                MetaState::LEADER_POSITION_X)= currentObservation.R();

    return true;
  }
  else
  {
    return false;
  }
}


}
