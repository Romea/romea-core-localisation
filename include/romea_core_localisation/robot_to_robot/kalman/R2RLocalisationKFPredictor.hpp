// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFPREDICTOR_HPP
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFPREDICTOR_HPP

// romea
#include "romea_core_localisation/LocalisationPredictor.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFMetaState.hpp"

namespace romea
{

class R2RLocalisationKFPredictor : public LocalisationPredictor<R2RLocalisationKFMetaState>
{
public:
  using MetaState = R2RLocalisationKFMetaState;
  using State = R2RLocalisationKFMetaState::State;
  using Input = R2RLocalisationKFMetaState::Input;
  using AddOn = R2RLocalisationKFMetaState::AddOn;

public:
  R2RLocalisationKFPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable);

private:
  bool stop_(
    const Duration & duration,
    const MetaState & metaState)override;

  void predict_(
    const MetaState & previousMetaState,
    MetaState & currentMetaState)override;

  void reset_(R2RLocalisationKFMetaState & metaState)override;

  void predictState_(
    const State & previousState,
    const Input & previousInput,
    State & currentState);

  void predictAddOn_(
    const AddOn & previousAddOn,
    const State & currentState,
    AddOn & currentAddOn);

private:
  Eigen::MatrixXd jFl_;
  Eigen::MatrixXd jGl_;
  Eigen::MatrixXd jFf_;
  Eigen::MatrixXd jGf_;

  double xl_, yl_, thetal_;
  double vxl_, vyl_, wl_;
  double vxldT_, vyldT_, wldT_;
  double dT_cos_thetal_wldT_;
  double dT_sin_thetal_wldT_;

  double vxf_, vyf_, wf_;
  double vxfdT_, vyfdT_, wfdT_;
  double dT_cos_wfdT_;
  double dT_sin_wfdT_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__R2RLOCALISATIONKFPREDICTOR_HPP
