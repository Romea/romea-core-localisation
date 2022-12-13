#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERPOSE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERPOSE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>
#include <romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp>

#include "romea_core_localisation/ObservationPose.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"

namespace romea {


class R2WLocalisationKFUpdaterPose :
  public LocalisationUpdaterExteroceptive,
  public KFUpdaterCore<double, 3, 3>
{
public :

  using Observation = ObservationPose;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterPose(const std::string & updaterName,
                               const double & minimalRate,
                               const TriggerMode & triggerMode,
                               const double &maximalMahalanobisDistance,
                               const std::string & logFilename);

  void update(const Duration & duration,
              const Observation  & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

private :


  void update_(const Duration & duration,
               const Observation  & currentObservation,
               State &currentState,
               AddOn &currentAddon);


  bool set_(const Duration & duration,
            const ObservationPose  & currentObservation,
            const Input &currentInput,
            State &currentState,
            AddOn &currentAddon);

  //  void applyLevelArmCompensation_(R2WLocalisationKFState & currentState,
  //                                  const Eigen::Vector3d & levelArm);

  LevelArmCompensation levelArmCompensation_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFUPDATERPOSE_HPP_





