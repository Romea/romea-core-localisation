// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONMETASTATE_HP
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONMETASTATE_HP

// romea
#include <romea_core_filtering/GaussianInput.hpp>
#include <romea_core_common/containers/Eigen/RingOfEigenVector.hpp>
#include "romea_core_localisation/LocalisationUpdatesMonitoring.hpp"

namespace romea
{

struct R2RLocalisationMetaState
{
  enum StateIndex
  {
    LEADER_POSITION_X = 0,
    LEADER_POSITION_Y,
    LEADER_ORIENTATION_Z,
    STATE_SIZE
  };

  enum InputIndex
  {
    LINEAR_SPEED_X_BODY = 0,
    LINEAR_SPEED_Y_BODY,
    ANGULAR_SPEED_Z_BODY,
    LEADER_LINEAR_SPEED_X_BODY,
    LEADER_LINEAR_SPEED_Y_BODY,
    LEADER_ANGULAR_SPEED_Z_BODY,
    INPUT_SIZE
  };

  using Input = GaussianInput<double, INPUT_SIZE>;

  struct AddOn
  {
    AddOn();

    void reset();

    LocalisationUpdateMonitoring lastExteroceptiveUpdate;
    RingOfEigenVector<Eigen::Vector2d> leaderTrajectory;
    RingOfEigenVector<Eigen::Vector2d> robotTrajectory;
    double travelledDistance;
  };

  R2RLocalisationMetaState();
  virtual ~R2RLocalisationMetaState() = default;

  Input input;
  AddOn addon;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONMETASTATE_HP
