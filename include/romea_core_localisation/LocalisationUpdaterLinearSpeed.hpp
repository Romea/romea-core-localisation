#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERLINEARSPEED_HPP_ 
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERLINEARSPEED_HPP_ 

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationLinearSpeed.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea {

template <class MetaState>
class LocalisationUpdaterLinearSpeed : public LocalisationUpdaterProprioceptive
{
public :

  LocalisationUpdaterLinearSpeed(const std::string & updaterName,
                                 const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  using Observation = ObservationLinearSpeed;

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template
      segment<2>(MetaState::LINEAR_SPEED_X_BODY) << currentObservation.Y(), 0;

    currentMetaState.input.QU().template
      block<2, 2>(MetaState::LINEAR_SPEED_X_BODY,
                  MetaState::LINEAR_SPEED_X_BODY) << currentObservation.R(), 0, 0, 0;
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERLINEARSPEED_HPP_ 
