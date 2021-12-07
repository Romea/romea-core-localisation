#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"
#include <romea_core_common/math/Matrix.hpp>

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationKFMetaState::R2WLocalisationKFMetaState():
  R2WLocalisationMetaState(),
  state()
{

}



//--------------------------------------------------------------------------
void applyLevelArmCompensation(R2WLocalisationKFMetaState::State & currentState,
                               R2WLocalisationKFMetaState::AddOn & currentAddOn,
                               LevelArmCompensation & levelArmCompensation,
                               const Eigen::Vector3d & levelArm)
{
  levelArmCompensation.compute(currentAddOn.roll,
                               currentAddOn.pitch,
                               currentAddOn.rollPitchVariance,
                               currentState.X(R2WLocalisationKFMetaState::ORIENTATION_Z),
                               0,
                               levelArm);

  currentState.X().segment<2>(R2WLocalisationKFMetaState::POSITION_X) -=
      levelArmCompensation.getPosition().segment<2>(R2WLocalisationKFMetaState::POSITION_X);

  currentState.P().block<2,2>(R2WLocalisationKFMetaState::POSITION_X,
                              R2WLocalisationKFMetaState::POSITION_X) +=
      levelArmCompensation.getPositionCovariance().block<2,2>(R2WLocalisationKFMetaState::POSITION_X,
                                                              R2WLocalisationKFMetaState::POSITION_X);

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}


}

