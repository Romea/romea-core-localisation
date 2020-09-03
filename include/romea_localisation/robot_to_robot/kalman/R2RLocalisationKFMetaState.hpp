#ifndef _R2RLocalisationKFMetaState_HPP_
#define _R2RLocalisationKFMetaState_HPP_


//romea
#include <romea_filtering/GaussianState.hpp>
#include "../R2RLocalisationMetaState.hpp"

namespace romea {

struct R2RLocalisationKFMetaState : public R2RLocalisationMetaState
{

  using State =GaussianState<double,STATE_SIZE>;

  R2RLocalisationKFMetaState();

  virtual ~R2RLocalisationKFMetaState()=default;

  State state;

};


}//romea
#endif
