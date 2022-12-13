#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFMETASTATE_HPP_


// romea
#include <romea_core_filtering/GaussianState.hpp>
#include "romea_core_localisation/robot_to_robot/R2RLocalisationMetaState.hpp"

namespace romea {

struct R2RLocalisationKFMetaState : public R2RLocalisationMetaState
{
  using State = GaussianState<double, STATE_SIZE>;

  R2RLocalisationKFMetaState();

  virtual ~R2RLocalisationKFMetaState() = default;

  State state;
};


}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFMETASTATE_HPP_
