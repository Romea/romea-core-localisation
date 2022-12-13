#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERRANGE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERRANGE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/kalman/UnscentedKalmanFilterUpdaterCore.hpp>
#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"

namespace romea {

class R2WLocalisationKFUpdaterRange :
  public LocalisationUpdaterExteroceptive,
  public UKFUpdaterCore<double, 3, 1>
{
public :

  using Observation = ObservationRange;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  void update_(const Duration & duration,
               const Observation &currentObservation,
               State &currentState,
               AddOn &currentAddon);

protected :

  LevelArmCompensation antennaAtitudeCompensation_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERRANGE_HPP_
