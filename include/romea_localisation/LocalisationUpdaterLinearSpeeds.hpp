#ifndef _romea_KalmanUpdaterLinearSpeeds_HPP_
#define _romea_KalmanUpdaterLinearSpeeds_HPP_


//local
#include <romea_common/time/Time.hpp>
#include "ObservationLinearSpeeds.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template < class MetaState>
class LocalisationUpdaterLinearSpeeds
{

public :

  using Observation = ObservationLinearSpeeds;

  LocalisationUpdaterLinearSpeeds()
  {
  }


  void update(const Duration & /*duration*/,
              const Observation &currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {

    currentMetaState.input.U().template segment<2>(MetaState::LINEAR_SPEED_X_BODY)=currentObservation.Y();

    currentMetaState.input.QU().template block<2,2>(MetaState::LINEAR_SPEED_X_BODY,
                                                    MetaState::LINEAR_SPEED_X_BODY)=currentObservation.R();

  }

};

}//romea

#endif
