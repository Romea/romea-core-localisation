// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_localisation/robot_to_human/R2HLocalisationMetaState.hpp"

namespace
{
const size_t MAXIMAL_TRAJECTORY_SIZE = 1000;
}

namespace romea
{

//-----------------------------------------------------------------------------
R2HLocalisationMetaState::AddOn::AddOn()
: lastExteroceptiveUpdate(),
  leaderTrajectory(MAXIMAL_TRAJECTORY_SIZE),
  robotTrajectory(MAXIMAL_TRAJECTORY_SIZE),
  travelledDistance(0)
{
}

//-----------------------------------------------------------------------------
void R2HLocalisationMetaState::AddOn::reset()
{
  lastExteroceptiveUpdate.time = Duration::max();
  leaderTrajectory.clear();
  robotTrajectory.clear();
  lastExteroceptiveUpdate.travelledDistance = 0;
  travelledDistance = 0;
}

//-----------------------------------------------------------------------------
R2HLocalisationMetaState::R2HLocalisationMetaState()
: input(),
  addon()
{
}

}  // namespace romea
