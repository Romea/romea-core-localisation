// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFPREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFPREDICTOR_HPP_

// romea
#include "romea_core_localisation/LocalisationPredictor.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"

namespace romea
{

class R2WLocalisationKFPredictor : public LocalisationPredictor<R2WLocalisationKFMetaState>
{
public:
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public:
  R2WLocalisationKFPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable);

protected:
  bool stop_(
    const Duration & duration,
    const MetaState & state)override;

  void predict_(
    const MetaState & previousState,
    MetaState & nextState)override;

  void reset_(MetaState & state)override;

  void predictState_(
    const State & previousState,
    const Input & previousInput,
    State & currentState);

  void predictAddOn_(
    const AddOn & previousAddOn,
    AddOn & currentAddOn);

private:
  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;

  double x_, y_, theta_, vx_, vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_theta_wdT_;
  double dT_sin_theta_wdT_;
};

}  // namespace romea

#endif   // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFPREDICTOR_HPP_
