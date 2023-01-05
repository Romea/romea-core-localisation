// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFPREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFPREDICTOR_HPP_

#include "romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFMetaState.hpp"
#include "romea_core_localisation/LocalisationPredictor.hpp"

namespace romea
{

class R2HLocalisationKFPredictor : public LocalisationPredictor<R2HLocalisationKFMetaState>
{
public:
  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public:
  R2HLocalisationKFPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable,
    const Eigen::Matrix2d & leaderMotionCovariance);

  virtual ~R2HLocalisationKFPredictor() = default;

private:
  bool stop_(
    const Duration & duration,
    const MetaState & state)override;

  void predict_(
    const MetaState & previousMetaState,
    MetaState & currentMetaState)override;

  void reset_(MetaState & metaState)override;

private:
  void predictState_(
    const State & previousState,
    const Input & prviousInput,
    State & currentState);

  void predictAddOn_(
    const AddOn & previousAddOn,
    const State & currentState,
    AddOn & currentAddOn);

private:
  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;
  Eigen::MatrixXd leaderMotionCovariance_;

  double vx_, vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_wdT_;
  double dT_sin_wdT_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFPREDICTOR_HPP
