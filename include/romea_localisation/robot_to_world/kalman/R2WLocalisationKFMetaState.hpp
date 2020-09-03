#ifndef romea_R2WLocalisationKFState_hpp
#define romea_R2WLocalisationKFState_hpp

// romea
#include <romea_filtering/GaussianState.hpp>
#include "../R2WLocalisationMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"

//std
#include <memory>

namespace romea {

struct R2WLocalisationKFMetaState : R2WLocalisationMetaState
{

  using State =GaussianState<double,STATE_SIZE>;

  R2WLocalisationKFMetaState();

  virtual ~R2WLocalisationKFMetaState()=default;

  State state;

};

void applyLevelArmCompensation(R2WLocalisationKFMetaState::State & currentState,
                               R2WLocalisationKFMetaState::AddOn & currentAddOn,
                               LevelArmCompensation & levelArmCompensation,
                               const Eigen::Vector3d & levelArm);

}


#endif
