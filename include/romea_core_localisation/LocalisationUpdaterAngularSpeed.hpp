// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERANGULARSPEED_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERANGULARSPEED_HPP_

// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationAngularSpeed.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea
{

template<class MetaState>
class LocalisationUpdaterAngularSpeed : public LocalisationUpdaterProprioceptive
{
public:
  using Observation = ObservationAngularSpeed;

  LocalisationUpdaterAngularSpeed(
    const std::string & updaterName,
    const double & minimalRate)
  : LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    LocalisationFSMState & /*currentFSMState*/,
    MetaState & currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = currentObservation.Y();

    currentMetaState.input.QU(
      MetaState::ANGULAR_SPEED_Z_BODY,
      MetaState::ANGULAR_SPEED_Z_BODY) = currentObservation.R();
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERANGULARSPEED_HPP_
