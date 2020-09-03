#ifndef _romea_KalmanUpdaterLinearSpeed_HPP_
#define _romea_KalmanUpdaterLinearSpeed_HPP_


//romea
#include <romea_common/time/Time.hpp>
#include "ObservationLinearSpeed.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterLinearSpeed
{

public :

  LocalisationUpdaterLinearSpeed()
  {
  }

  using Observation = ObservationLinearSpeed;

  void update(const Duration & /*duration*/,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {

    currentMetaState.input.U().template segment<2>(MetaState::LINEAR_SPEED_X_BODY)<<currentObservation.Y(),0;

    currentMetaState.input.QU().template block<2,2>(MetaState::LINEAR_SPEED_X_BODY,
                                                    MetaState::LINEAR_SPEED_X_BODY)<<currentObservation.R(),0,0,0;
  }

};

}//romea

#endif
