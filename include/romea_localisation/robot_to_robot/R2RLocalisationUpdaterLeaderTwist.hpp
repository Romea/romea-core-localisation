#ifndef _romea_R2RLocalisationUpdaterTwist_HPP_
#define _romea_R2RLocalisationUpdaterTwist_HPP_


//local
#include <romea_common/time/Time.hpp>
#include "../ObservationTwist.hpp"
#include "../LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class R2RLocalisationUpdaterLeaderTwist
{

public :

  R2RLocalisationUpdaterLeaderTwist()
  {

  }

  using Observation = ObservationTwist;

  virtual void update(const Duration & /*duration*/,
                      const ObservationTwist & currentObservation,
                      LocalisationFSMState & /*currentFSMState*/,
                      MetaState &currentMetaState)
  {

    currentMetaState.input.U().template segment<3>(MetaState::LEADER_LINEAR_SPEED_X_BODY)=currentObservation.Y();

    currentMetaState.input.QU().template block<3,3>(MetaState::LEADER_LINEAR_SPEED_X_BODY,
                                                    MetaState::LEADER_LINEAR_SPEED_X_BODY)=currentObservation.R();
  }

};

}//romea

#endif