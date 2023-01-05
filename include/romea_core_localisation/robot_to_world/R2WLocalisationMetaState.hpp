// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONMETASTATE_HPP_

// romea
#include <romea_core_filtering/GaussianInput.hpp>
#include <romea_core_common/containers/Eigen/RingOfEigenVector.hpp>
#include "../LocalisationUpdatesMonitoring.hpp"

namespace romea
{

struct R2WLocalisationMetaState
{
  enum StateIndex
  {
    POSITION_X = 0,
    POSITION_Y,
    ORIENTATION_Z,
    STATE_SIZE
  };

  enum InputIndex
  {
    LINEAR_SPEED_X_BODY = 0,
    LINEAR_SPEED_Y_BODY,
    ANGULAR_SPEED_Z_BODY,
    INPUT_SIZE
  };

  using Input = GaussianInput<double, INPUT_SIZE>;

  struct AddOn
  {
    AddOn();

    void reset();

    LocalisationUpdateMonitoring lastExteroceptiveUpdate;

    double roll;
    double pitch;
    double rollPitchVariance;
    double travelledDistance;
  };

  R2WLocalisationMetaState();
  virtual ~R2WLocalisationMetaState() = default;

  Input input;
  AddOn addon;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONMETASTATE_HPP_
