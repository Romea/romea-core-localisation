#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERTWIST_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERTWIST_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationTwist.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterTwist : public LocalisationUpdaterProprioceptive
{

public :

  LocalisationUpdaterTwist(const std::string & updaterName,
                           const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  using Observation = ObservationTwist;

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template
      segment<3>(MetaState::LINEAR_SPEED_X_BODY) = currentObservation.Y();

    currentMetaState.input.QU().template
      block<3, 3>(MetaState::LINEAR_SPEED_X_BODY,
                  MetaState::LINEAR_SPEED_X_BODY) = currentObservation.R();
  }
};

}  //namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERTWIST_HPP_
