// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFMETASTATE_HPP_

// romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>
#include "romea_core_localisation/robot_to_robot/R2RLocalisationMetaState.hpp"

namespace romea
{

struct R2RLocalisationPFMetaState : R2RLocalisationMetaState
{
  explicit R2RLocalisationPFMetaState(const size_t & numberOfParticles);

  virtual ~R2RLocalisationPFMetaState() = default;

  using State = ParticleFilterState<double, STATE_SIZE>;

  State state;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFMETASTATE_HPP_
