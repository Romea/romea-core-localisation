// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERLINEARSPEEDS_HPP
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERLINEARSPEEDS_HPP


// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"
#include "romea_core_localisation/ObservationLinearSpeeds.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea
{

template<class MetaState>
class LocalisationUpdaterLinearSpeeds : public LocalisationUpdaterProprioceptive
{
public:
  using Observation = ObservationLinearSpeeds;

  LocalisationUpdaterLinearSpeeds(
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

    currentMetaState.input.U().template
    segment<2>(MetaState::LINEAR_SPEED_X_BODY) = currentObservation.Y();

    currentMetaState.input.QU().template
    block<2, 2>(
      MetaState::LINEAR_SPEED_X_BODY,
      MetaState::LINEAR_SPEED_X_BODY) = currentObservation.R();
  }
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERLINEARSPEEDS_HPP
