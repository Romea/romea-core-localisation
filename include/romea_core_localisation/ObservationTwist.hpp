// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__OBSERVATIONTWIST_HPP_
#define ROMEA_CORE_LOCALISATION__OBSERVATIONTWIST_HPP_

// romea
#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea
{

struct ObservationTwist : GaussianObservation<double, 3>
{
  enum Index
  {
    LINEAR_SPEED_X_BODY = 0,
    LINEAR_SPEED_Y_BODY,
    ANGULAR_SPEED_Z_BODY,
    SIZE
  };

  Eigen::Vector3d levelArm;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__OBSERVATIONTWIST_HPP_
