#ifndef _romea_LocalisationUpdaterAngularSpeed_HPP_
#define _romea_LocalisationUpdaterAngularSpeed_HPP_


//local
#include <romea_common/time/Time.hpp>
#include <romea_filtering/FilterUpdater.hpp>
#include "ObservationAngularSpeed.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template < class MetaState>
class LocalisationUpdaterAngularSpeed
{

public :

  using Observation = ObservationAngularSpeed;

  LocalisationUpdaterAngularSpeed()
  {
  }

  void update(const Duration & /*duration*/,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState & currentMetaState)
  {

    currentMetaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY)=currentObservation.Y();

    currentMetaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                              MetaState::ANGULAR_SPEED_Z_BODY)=currentObservation.R();

  }

};

}//romea

#endif
