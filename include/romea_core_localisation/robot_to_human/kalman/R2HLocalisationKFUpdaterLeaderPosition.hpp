#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFUPDATERLEADERPOSITION_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFUPDATERLEADERPOSITION_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/ObservationPosition.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFMetaState.hpp"

namespace romea {


class R2HLocalisationKFUpdaterLeaderPosition : public LocalisationUpdaterExteroceptive
{
public :

  using Observation = ObservationPosition;
  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public :

  R2HLocalisationKFUpdaterLeaderPosition(const std::string & updaterName,
                                         const double & minimalRate,
                                         const TriggerMode & triggerMode,
                                         const double & maximalMahalanobisDistance,
                                         const std::string & logFilename);

  void update(const Duration & duration,
              const Observation &currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);


  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFUPDATERLEADERPOSITION_HPP_
