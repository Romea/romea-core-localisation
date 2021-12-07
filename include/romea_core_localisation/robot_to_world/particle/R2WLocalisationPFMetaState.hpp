#ifndef romea_R2WParticleLocalisationState_hpp
#define romea_R2WParticleLocalisationState_hpp

//romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>
#include "../R2WLocalisationMetaState.hpp"

//std
#include <random>


namespace romea {

struct R2WLocalisationPFMetaState : R2WLocalisationMetaState
{

public :

  using State =ParticleFilterState<double,STATE_SIZE>;

  R2WLocalisationPFMetaState(const size_t & numberOfParticles);

  virtual ~R2WLocalisationPFMetaState()=default;

  State state;
};

}


#endif
