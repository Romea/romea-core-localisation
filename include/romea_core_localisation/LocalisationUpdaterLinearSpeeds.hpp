#ifndef _romea_LocalisationUpdaterLinearSpeeds_HPP_
#define _romea_LocalisationUpdaterLinearSpeeds_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationLinearSpeeds.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea {

template < class MetaState>
class LocalisationUpdaterLinearSpeeds : public LocalisationUpdaterProprioceptive
{

public :

  using Observation = ObservationLinearSpeeds;

  LocalisationUpdaterLinearSpeeds(const std::string & updaterName,
                                  const double & minimalRate):
    LocalisationUpdaterProprioceptive (updaterName, minimalRate)
  {
  }


  void update(const Duration & duration,
              const Observation &currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template
      segment<2>(MetaState::LINEAR_SPEED_X_BODY)= currentObservation.Y();

    currentMetaState.input.QU().template
      block<2, 2>(MetaState::LINEAR_SPEED_X_BODY,
                  MetaState::LINEAR_SPEED_X_BODY)= currentObservation.R();
  }

};

}//romea

#endif
