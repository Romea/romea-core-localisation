#ifndef _romea_R2HLocalisationKFState_HPP_
#define _romea_R2HLocalisationKFState_HPP_

//romea
#include <romea_core_filtering/GaussianState.hpp>
#include "../R2HLocalisationMetaState.hpp"

namespace romea {


struct R2HLocalisationKFMetaState : R2HLocalisationMetaState
{

  using State =GaussianState<double,STATE_SIZE>;

  R2HLocalisationKFMetaState();

  virtual ~R2HLocalisationKFMetaState()=default;

  State state;

};

}//romea

#endif
