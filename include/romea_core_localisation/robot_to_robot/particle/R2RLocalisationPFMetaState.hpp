#ifndef romea_R2RLocalisationPFMetaState_hpp
#define romea_R2RLocalisationPFMetaState_hpp

//romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>
#include "../R2RLocalisationMetaState.hpp"

namespace romea {

struct R2RLocalisationPFMetaState : R2RLocalisationMetaState
{

  R2RLocalisationPFMetaState(const size_t & numberOfParticles);

  using State =ParticleFilterState<double,STATE_SIZE>;

  R2RLocalisationPFMetaState();

  virtual ~R2RLocalisationPFMetaState()=default;

  State state;

};

}


#endif
