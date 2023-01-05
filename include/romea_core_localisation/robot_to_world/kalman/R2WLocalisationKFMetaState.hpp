// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_


// romea
#include <romea_core_filtering/GaussianState.hpp>

// std
#include <memory>

// local
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

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_
