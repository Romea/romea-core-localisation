// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFMETASTATE_HPP_

// namespace romea
#include <romea_core_filtering/GaussianState.hpp>
#include "romea_core_localisation/robot_to_human/R2HLocalisationMetaState.hpp"

namespace romea
{

struct R2HLocalisationKFMetaState : R2HLocalisationMetaState
{
  using State = GaussianState<double, STATE_SIZE>;

  R2HLocalisationKFMetaState();

  virtual ~R2HLocalisationKFMetaState() = default;

  State state;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFMETASTATE_HPP_
