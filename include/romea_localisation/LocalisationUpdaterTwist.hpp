#ifndef _romea_LocalisationUpdaterTwist_HPP_
#define _romea_LocalisationUpdaterTwist_HPP_


//local
#include <romea_common/time/Time.hpp>
#include "ObservationTwist.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterTwist
{

public :

  LocalisationUpdaterTwist()
  {
  }

  using Observation = ObservationTwist;

  void update(const Duration & /*duration*/,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {

    currentMetaState.input.U().template segment<3>(MetaState::LINEAR_SPEED_X_BODY)=currentObservation.Y();

    currentMetaState.input.QU().template block<3,3>(MetaState::LINEAR_SPEED_X_BODY,
                                                    MetaState::LINEAR_SPEED_X_BODY)=currentObservation.R();
  }

};

}//romea

#endif
