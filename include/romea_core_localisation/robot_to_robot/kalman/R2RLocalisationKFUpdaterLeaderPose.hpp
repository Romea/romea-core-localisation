#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFUPDATERLEADERPOSE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFUPDATERLEADERPOSE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/ObservationPose.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFMetaState.hpp"

namespace romea {

class R2RLocalisationKFUpdaterLeaderPose : public LocalisationUpdaterExteroceptive
{
public :

  using Observation = ObservationPose;
  using MetaState = R2RLocalisationKFMetaState;
  using State = R2RLocalisationKFMetaState::State;
  using Input = R2RLocalisationKFMetaState::Input;
  using AddOn = R2RLocalisationKFMetaState::AddOn;

public :

  R2RLocalisationKFUpdaterLeaderPose(const std::string & updaterName,
                                     const double & minimalRate,
                                     const TriggerMode & triggerMode,
                                     const double & maximalMahalanobisDistance,
                                     const std::string & logFilename);

  void update(const Duration & duration,
              const Observation &currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFUPDATERLEADERPOSE_HPP_
