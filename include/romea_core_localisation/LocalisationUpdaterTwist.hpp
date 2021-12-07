#ifndef _romea_LocalisationUpdaterTwist_HPP_
#define _romea_LocalisationUpdaterTwist_HPP_


//local
#include <romea_core_common/time/Time.hpp>
#include "LocalisationUpdaterProprioceptive.hpp"
#include "ObservationTwist.hpp"
#include "LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterTwist : public LocalisationUpdaterProprioceptive
{

public :

  LocalisationUpdaterTwist(const std::string & updaterName,
                           const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName,minimalRate)
  {
  }

  using Observation = ObservationTwist;

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {

    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template segment<3>(MetaState::LINEAR_SPEED_X_BODY)=currentObservation.Y();

    currentMetaState.input.QU().template block<3,3>(MetaState::LINEAR_SPEED_X_BODY,
                                                    MetaState::LINEAR_SPEED_X_BODY)=currentObservation.R();
  }

};

}//romea

#endif
