// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP


// romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>

// std
#include <random>

// local
#include "../R2WLocalisationMetaState.hpp"

namespace romea
{

struct R2WLocalisationPFMetaState : R2WLocalisationMetaState
{
public:
  using State = ParticleFilterState<double, STATE_SIZE>;

  explicit R2WLocalisationPFMetaState(const size_t & numberOfParticles);

  virtual ~R2WLocalisationPFMetaState() = default;

  State state;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP
