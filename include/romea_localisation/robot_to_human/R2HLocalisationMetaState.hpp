#ifndef _romea_R2HLocalisationState_HPP_
#define _romea_R2HLocalisationState_HPP_

//romea
#include <romea_filtering/GaussianInput.hpp>
#include <romea_common/containers/Eigen/RingOfEigenVector.hpp>
#include "../LocalisationUpdatesMonitoring.hpp"

namespace romea {

struct R2HLocalisationMetaState
{

  enum StateIndex {
    LEADER_POSITION_X = 0,
    LEADER_POSITION_Y,
    STATE_SIZE
  };

  enum InputIndex {
    LINEAR_SPEED_X_BODY = 0,
    LINEAR_SPEED_Y_BODY,
    ANGULAR_SPEED_Z_BODY,
    INPUT_SIZE
  };

  using Input = GaussianInput<double,INPUT_SIZE>;

  struct AddOn
  {
    AddOn();

    void reset();

    LocalisationUpdateMonitoring lastExteroceptiveUpdate;
    RingOfEigenVector<Eigen::Vector2d> leaderTrajectory;
    RingOfEigenVector<Eigen::Vector2d> robotTrajectory;
    double travelledDistance;
  };

  R2HLocalisationMetaState();
  virtual ~R2HLocalisationMetaState()=default;

  Input input;
  AddOn addon;
};

}//romea

#endif
