// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea
{

//--------------------------------------------------------------------------
R2WLocalisationPFMetaState::R2WLocalisationPFMetaState(const size_t & numberOfParticles)
: R2WLocalisationMetaState(),
  state(numberOfParticles)
{
}

}  // namespace romea
