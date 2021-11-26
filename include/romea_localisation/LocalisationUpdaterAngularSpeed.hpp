#ifndef _romea_LocalisationUpdaterAngularSpeed_HPP_
#define _romea_LocalisationUpdaterAngularSpeed_HPP_


//local
#include <romea_common/time/Time.hpp>
#include "LocalisationUpdaterProprioceptive.hpp"
#include "ObservationAngularSpeed.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template < class MetaState>
class LocalisationUpdaterAngularSpeed : public LocalisationUpdaterProprioceptive
{

public :

  using Observation = ObservationAngularSpeed;

  LocalisationUpdaterAngularSpeed(const std::string & updaterName,
                                  const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName,minimalRate)
  {
  }

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState & currentMetaState)
  {

    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY)=currentObservation.Y();

    currentMetaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                              MetaState::ANGULAR_SPEED_Z_BODY)=currentObservation.R();

  }

};

}//romea

#endif
