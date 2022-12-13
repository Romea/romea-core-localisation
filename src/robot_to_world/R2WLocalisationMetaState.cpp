// romea
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

namespace romea {

//-----------------------------------------------------------------------------
R2WLocalisationMetaState::AddOn::AddOn():
  lastExteroceptiveUpdate(),
  roll(0),
  pitch(0),
  rollPitchVariance(0),
  travelledDistance(0)
{
}

//-----------------------------------------------------------------------------
void R2WLocalisationMetaState::AddOn::reset()
{
  lastExteroceptiveUpdate.time = Duration::max();
  lastExteroceptiveUpdate.travelledDistance = 0;
  roll = 0;
  pitch = 0;
  rollPitchVariance = 0;
  travelledDistance = 0;
}

//-----------------------------------------------------------------------------
R2WLocalisationMetaState::R2WLocalisationMetaState():
  input(),
  addon()
{
}

}  // namespace romea

