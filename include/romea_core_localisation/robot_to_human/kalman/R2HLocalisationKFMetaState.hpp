#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFMETASTATE_HPP_  
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFMETASTATE_HPP_  

// namespace romea
#include <romea_core_filtering/GaussianState.hpp>
#include "romea_core_localisation/robot_to_human/R2HLocalisationMetaState.hpp"

namespace romea {

struct R2HLocalisationKFMetaState : R2HLocalisationMetaState
{
  using State = GaussianState<double, STATE_SIZE>;

  R2HLocalisationKFMetaState();

  virtual ~R2HLocalisationKFMetaState() = default;

  State state;
};

}  //namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_KALMAN_R2HLOCALISATIONKFMETASTATE_HPP_  
