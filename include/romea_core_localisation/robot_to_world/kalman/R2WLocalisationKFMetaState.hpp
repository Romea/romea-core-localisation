#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFMETASTATE_HPP_

// std
#include <memory>

// romea
#include <romea_core_filtering/GaussianState.hpp>
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"


namespace romea {

struct R2WLocalisationKFMetaState : R2WLocalisationMetaState
{
  using State = GaussianState<double, STATE_SIZE>;

  R2WLocalisationKFMetaState();

  virtual ~R2WLocalisationKFMetaState() = default;

  State state;
};

void applyLevelArmCompensation(R2WLocalisationKFMetaState::State & currentState,
                               R2WLocalisationKFMetaState::AddOn & currentAddOn,
                               LevelArmCompensation & levelArmCompensation,
                               const Eigen::Vector3d & levelArm);

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFMETASTATE_HPP_
