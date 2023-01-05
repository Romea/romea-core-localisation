// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFUPDATERRANGE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFUPDATERRANGE_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/kalman/UnscentedKalmanFilterUpdaterCore.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFMetaState.hpp"


namespace romea
{

class R2RLocalisationKFUpdaterRange
  : public LocalisationUpdaterExteroceptive,
  public UKFUpdaterCore<double, 3, 1>
{
public:
  using Observation = ObservationRange;
  using MetaState = R2RLocalisationKFMetaState;
  using State = R2RLocalisationKFMetaState::State;
  using Input = R2RLocalisationKFMetaState::Input;
  using AddOn = R2RLocalisationKFMetaState::AddOn;

public:
  R2RLocalisationKFUpdaterRange(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const double & maximalMahalanobisDistance,
    const std::string & logFilename);

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    LocalisationFSMState & currentFSMState,
    MetaState & currentState);

protected:
  void update_(
    const Duration & duration,
    const Observation & currentObservation,
    State & currentState,
    AddOn & currentAddOn);
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFUPDATERRANGE_HPP_
