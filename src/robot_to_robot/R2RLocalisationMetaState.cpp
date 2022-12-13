// romea
#include "romea_core_localisation/robot_to_robot/R2RLocalisationMetaState.hpp"

namespace {
const size_t MAXIMAL_TRAJECTORY_SIZE = 1000;
}

namespace romea {

//-----------------------------------------------------------------------------
R2RLocalisationMetaState::AddOn::AddOn():
  lastExteroceptiveUpdate(),
  leaderTrajectory(MAXIMAL_TRAJECTORY_SIZE),
  robotTrajectory(MAXIMAL_TRAJECTORY_SIZE),
  travelledDistance(0)
{

}

//-----------------------------------------------------------------------------
void R2RLocalisationMetaState::AddOn::reset()
{
  lastExteroceptiveUpdate.time = Duration::max();
  leaderTrajectory.clear();
  robotTrajectory.clear();
  lastExteroceptiveUpdate.travelledDistance = 0;
}

//-----------------------------------------------------------------------------
R2RLocalisationMetaState::R2RLocalisationMetaState():
  input(),
  addon()
{
}

}  // namespace romea

