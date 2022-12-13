#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFMETASTATE_HPP
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFMETASTATE_HPP

// std
#include <random>

// romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>
#include "../R2WLocalisationMetaState.hpp"

namespace romea {

struct R2WLocalisationPFMetaState : R2WLocalisationMetaState
{
public :

  using State = ParticleFilterState<double, STATE_SIZE>;

  explicit R2WLocalisationPFMetaState(const size_t & numberOfParticles);

  virtual ~R2WLocalisationPFMetaState() = default;

  State state;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFMETASTATE_HPP
