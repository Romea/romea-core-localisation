#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERANGULARSPEED_HPP_ 
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERANGULARSPEED_HPP_ 

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationAngularSpeed.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea {

template < class MetaState>
class LocalisationUpdaterAngularSpeed : public LocalisationUpdaterProprioceptive
{
public :

  using Observation = ObservationAngularSpeed;

  LocalisationUpdaterAngularSpeed(const std::string & updaterName,
                                  const double & minimalRate):
    LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState & currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = currentObservation.Y();

    currentMetaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                              MetaState::ANGULAR_SPEED_Z_BODY) = currentObservation.R();
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERANGULARSPEED_HPP_
