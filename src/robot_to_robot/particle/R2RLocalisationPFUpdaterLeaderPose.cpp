//romea
#include "romea_localisation/robot_to_robot/particle/R2RLocalisationPFUpdaterLeaderPose.hpp"
#include <romea_common/math/NormalRandomMatrixGenerator.hpp>

namespace romea {

//-----------------------------------------------------------------------------
R2RLocalisationPFUpdaterLeaderPose::R2RLocalisationPFUpdaterLeaderPose()
{
}


//--------------------------------------------------------------------------
void R2RLocalisationPFUpdaterLeaderPose::update(const Duration &duration,
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
bool R2RLocalisationPFUpdaterLeaderPose::set_(const Duration & duration,
                                              const Observation &currentObservation,
                                              const Input &currentInput,
                                              State & currentState,
                                              AddOn & currentAddOn)
{

  if(!std::isnan(currentInput.U(R2RLocalisationPFMetaState::LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LINEAR_SPEED_Y_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationPFMetaState::ANGULAR_SPEED_Z_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(R2RLocalisationPFMetaState::LEADER_LINEAR_SPEED_X_BODY)))
  {

    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(currentObservation.Y(),currentObservation.R());
    randomGenerator.fill(currentState.particles);

    currentAddOn.lastExteroceptiveUpdate.time=duration;
    currentAddOn.lastExteroceptiveUpdate.travelledDistance = currentAddOn.travelledDistance;

    assert(!std::isnan(currentState.particles(0,0)));
    assert(!std::isnan(currentState.particles(0,1)));
    assert(!std::isnan(currentState.particles(0,2)));

    return true;
  }
  else
  {
    return false;
  }
}

}





