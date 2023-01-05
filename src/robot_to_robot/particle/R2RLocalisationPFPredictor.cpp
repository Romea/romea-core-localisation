// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFPredictor.hpp"
#include <romea_core_common/containers/Eigen/EigenContainers.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>

namespace romea
{

//--------------------------------------------------------------------------
R2RLocalisationPFPredictor::R2RLocalisationPFPredictor(
  const Duration & maximalDurationInDeadReckoning,
  const double & maximalTravelledDistanceInDeadReckoning,
  const double & maximalPositionCircularErrorProbable,
  const size_t & numberOfParticles)
: LocalisationPredictor<MetaState>(maximalDurationInDeadReckoning,
    maximalTravelledDistanceInDeadReckoning,
    maximalPositionCircularErrorProbable),
  cosCourses_(RowMajorVector::Zero(numberOfParticles)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles)),
  wfdT_(0),
  vxfdT_(0),
  vyfdT_(0),
  Uf_(Eigen::Vector3d::Zero()),
  QUf_(Eigen::Matrix3d::Zero()),
  Ufinv_(Eigen::Vector3d::Zero()),
  QUfinv_(Eigen::Matrix3d::Zero()),
  randomUfinv_(RowMajorMatrix::Zero(3, numberOfParticles)),
  Ul_(Eigen::Vector3d::Zero()),
  QUl_(Eigen::Matrix3d::Zero()),
  randomUl_(RowMajorMatrix::Zero(3, numberOfParticles))
{
}


//--------------------------------------------------------------------------
void R2RLocalisationPFPredictor::predict_(
  const MetaState & previousMetaState,
  MetaState & currentMetaState)
{
  currentMetaState.input = previousMetaState.input;

  predictState_(
    previousMetaState.state,
    previousMetaState.input,
    currentMetaState.state);

  predictAddOn_(
    previousMetaState.addon,
    currentMetaState.state,
    currentMetaState.addon);

  assert(isPositiveSemiDefiniteMatrix(currentMetaState.input.QU()));
}


//-----------------------------------------------------------------------------
void R2RLocalisationPFPredictor::drawFollowerInputs(const Input & previousInput)
{
  wfdT_ = previousInput.U(MetaState::ANGULAR_SPEED_Z_BODY) * dt_;
  vxfdT_ = previousInput.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) * dt_;
  vyfdT_ = previousInput.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) * dt_;

  Uf_ = previousInput.U().segment<3>(MetaState::LINEAR_SPEED_X_BODY) * dt_;

  QUf_ = previousInput.QU().block<3, 3>(
    MetaState::LINEAR_SPEED_X_BODY,
    MetaState::LINEAR_SPEED_X_BODY) * dt_ * dt_;

  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0, 0) = -std::cos(wfdT_);
  J(0, 1) = -std::sin(wfdT_);
  J(1, 0) = std::sin(wfdT_);
  J(1, 1) = -std::cos(wfdT_);
  J(2, 2) = -1;

  Ufinv_ = J * Uf_;
  QUfinv_ = J * QUf_ * J.transpose();

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(Ufinv_, QUfinv_);
  randomGenerator.fill(randomUfinv_);
}


//-----------------------------------------------------------------------------
void R2RLocalisationPFPredictor::drawLeaderInputs(const Input & previousInput)
{
  Ul_ = previousInput.U().segment<3>(MetaState::LEADER_LINEAR_SPEED_X_BODY) * dt_;

  QUl_ = previousInput.QU().block<3, 3>(
    MetaState::LEADER_LINEAR_SPEED_X_BODY,
    MetaState::LEADER_LINEAR_SPEED_X_BODY) * dt_ * dt_;

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(Ul_, QUl_);
  randomGenerator.fill(randomUl_);
}

//-----------------------------------------------------------------------------
void R2RLocalisationPFPredictor::predictState_(
  const State & previousState,
  const Input & previousInput,
  State & currentState)
{
  drawFollowerInputs(previousInput);
  drawLeaderInputs(previousInput);

  // predict particles
  currentState.particles.row(MetaState::LEADER_ORIENTATION_Z) =
    previousState.particles.row(MetaState::LEADER_ORIENTATION_Z) +
    randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUl_.row(MetaState::ANGULAR_SPEED_Z_BODY);

  cosCourses_ = currentState.particles.row(MetaState::LEADER_ORIENTATION_Z).cos();
  sinCourses_ = currentState.particles.row(MetaState::LEADER_ORIENTATION_Z).sin();

  currentState.particles.row(MetaState::LEADER_POSITION_X) =
    previousState.particles.row(MetaState::LEADER_POSITION_X) -
    previousState.particles.row(MetaState::LEADER_POSITION_Y) *
    randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUfinv_.row(MetaState::LINEAR_SPEED_X_BODY) +
    cosCourses_ * randomUl_.row(MetaState::LINEAR_SPEED_X_BODY) -
    sinCourses_ * randomUl_.row(MetaState::LINEAR_SPEED_Y_BODY);

  currentState.particles.row(MetaState::LEADER_POSITION_Y) =
    previousState.particles.row(MetaState::LEADER_POSITION_Y) +
    previousState.particles.row(MetaState::LEADER_POSITION_X) *
    randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUfinv_.row(MetaState::LINEAR_SPEED_Y_BODY) +
    sinCourses_ * randomUl_.row(MetaState::LINEAR_SPEED_X_BODY) *
    cosCourses_ * randomUl_.row(MetaState::LINEAR_SPEED_Y_BODY);

  currentState.weights = previousState.weights;
}

//-----------------------------------------------------------------------------
void R2RLocalisationPFPredictor::predictAddOn_(
  const AddOn & previousAddOn,
  const State & currentState,
  AddOn & currentAddOn)
{
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wfdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxfdT_, vyfdT_);

  // Predict additional data
  transform(
    previousAddOn.robotTrajectory.get(),
    currentAddOn.robotTrajectory.get(),
    R, T);

  transform(
    previousAddOn.leaderTrajectory.get(),
    currentAddOn.leaderTrajectory.get(),
    R, T);

  currentAddOn.robotTrajectory.ringIndex_ = previousAddOn.robotTrajectory.ringIndex_;
  currentAddOn.leaderTrajectory.ringIndex_ = previousAddOn.leaderTrajectory.ringIndex_;

  if (currentAddOn.robotTrajectory.size() == 0 ||
    currentAddOn.robotTrajectory[0].norm() > 0.1)
  {
    currentAddOn.robotTrajectory.append(Eigen::Vector2d::Zero());
    double x =
      (currentState.particles.row(MetaState::LEADER_POSITION_X) * currentState.weights).sum();
    double y =
      (currentState.particles.row(MetaState::LEADER_POSITION_Y) * currentState.weights).sum();
    currentAddOn.leaderTrajectory.append(Eigen::Vector2d(x, y));
  }

  currentAddOn.lastExteroceptiveUpdate = previousAddOn.lastExteroceptiveUpdate;
  currentAddOn.travelledDistance = previousAddOn.travelledDistance +
    std::sqrt(vxfdT_ * vxfdT_ + vyfdT_ * vyfdT_);
}

//-----------------------------------------------------------------------------
bool R2RLocalisationPFPredictor::stop_(
  const Duration & duration,
  const MetaState & meatastate)
{
  Duration durationInDeadReckoningMode = duration - meatastate.addon.lastExteroceptiveUpdate.time;

  double travelledDistanceInDeadReckoningMode =
    meatastate.addon.travelledDistance - meatastate.addon.lastExteroceptiveUpdate.travelledDistance;


  //  double positionCircularErrorProbability = std::sqrt(state.P(R2RKalmanLocalisationState::POSITION_X,
  //                                                              R2RKalmanLocalisationState::POSITION_X)+
  //                                                      state.P(R2RKalmanLocalisationState::POSITION_Y,
  //                                                              R2RKalmanLocalisationState::POSITION_Y));

  double positionCircularErrorProbability = 0;

  // std::cout << " particle dr elapsed time "<< durationToSecond(duration) <<" "<< durationToSecond(state.lastExteroceptiveUpdate.time) <<" " <<  durationToSecond(shutoffParameters_.maximalDurationInDeadReckoning)<< std::endl;
  // std::cout << " particle dr elapsed distance "<< state.travelledDistance <<" " <<state.lastExteroceptiveUpdate.travelledDistance <<" " <<  shutoffParameters_.maximalTravelledDistanceInDeadReckoning<< std::endl;

  return positionCircularErrorProbability > maximalPositionCircularErrorProbable_ ||
         travelledDistanceInDeadReckoningMode > maximalTravelledDistanceInDeadReckoning_ ||
         durationInDeadReckoningMode > maximalDurationInDeadReckoning_;
}

//-----------------------------------------------------------------------------
void R2RLocalisationPFPredictor::reset_(MetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace romea
