#ifndef _romea_LocalisationUpdaterLinearSpeed_HPP_
#define _romea_LocalisationKalmanUpdaterLinearSpeed_HPP_


//romea
#include <romea_common/time/Time.hpp>
#include "LocalisationUpdaterProprioceptive.hpp"
#include "ObservationLinearSpeed.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterLinearSpeed : public LocalisationUpdaterProprioceptive
{

public :

  LocalisationUpdaterLinearSpeed(const std::string & updaterName,
                                 const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName,minimalRate)
  {
  }

  using Observation = ObservationLinearSpeed;

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {

    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template segment<2>(MetaState::LINEAR_SPEED_X_BODY)<<currentObservation.Y(),0;

    currentMetaState.input.QU().template block<2,2>(MetaState::LINEAR_SPEED_X_BODY,
                                                    MetaState::LINEAR_SPEED_X_BODY)<<currentObservation.R(),0,0,0;
  }

};

}//romea

#endif
