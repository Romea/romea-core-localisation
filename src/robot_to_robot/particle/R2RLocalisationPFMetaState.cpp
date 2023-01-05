// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFMetaState.hpp"

namespace romea
{

//--------------------------------------------------------------------------
R2RLocalisationPFMetaState::R2RLocalisationPFMetaState(const size_t & numberOfParticles)
: R2RLocalisationMetaState(),
  state(numberOfParticles)
{
}

}  // namespace romea
