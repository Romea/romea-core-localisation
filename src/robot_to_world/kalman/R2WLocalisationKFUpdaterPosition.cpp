// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// local
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterPosition.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
R2WLocalisationKFUpdaterPosition::R2WLocalisationKFUpdaterPosition(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: LocalisationUpdaterExteroceptive(updaterName,
    minimalRate,
    triggerMode,
    logFilename),
  KFUpdaterCore(maximalMahalanobisDistance),
  levelArmCompensation_()
{
  this->H_(0, MetaState::POSITION_X) = 1;
  this->H_(1, MetaState::POSITION_Y) = 1;

  setLogFileHeader_(
    {"stamp",
      "x_obs",
      "y_obs",
      "cov_x_obs",
      "cov_y_obs",
      "cov_xy_obs",
      "x",
      "y",
      "theta",
      "cov_x",
      "cov_xy",
      "cov_xtheta",
      "cov_y",
      "cov_ytheta",
      "cov_theta",
      "roll",
      "pitch"
      "level_arm_x",
      "level_arm_y",
      "level_arm_z",
      "x_ant_a_priori",
      "y_ant_a_priori",
      "inn_x",
      "inn_y",
      "mahalanobis_distance",
      "success"
    });
}

//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterPosition::update(
  const Duration & duration,
  const Observation & currentObservation,
  LocalisationFSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rateDiagnostic_.evaluate(duration);

  switch (currentFSMState) {
    case LocalisationFSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        std::cout << " FSM : INIT DONE (POSITION + COURSE), GO TO RUNNING MODE " << std::endl;
        currentFSMState = LocalisationFSMState::RUNNING;
      }
      break;
    case LocalisationFSMState::RUNNING:
      if (triggerMode_ == TriggerMode::ALWAYS) {
        try {
          update_(
            duration,
            currentObservation,
            currentMetaState.state,
            currentMetaState.addon);
        } catch (...) {
          std::cout << " FSM : POSITION UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
          currentFSMState = LocalisationFSMState::INIT;
          currentMetaState.state.reset();
          currentMetaState.addon.reset();
        }
      }
      break;
    default:
      break;
  }
}


//-----------------------------------------------------------------------------
void R2WLocalisationKFUpdaterPosition::update_(
  const Duration & duration,
  Observation const & currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  State previousState = currentState;

  // compute antenna attitude compensation
  levelArmCompensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.rollPitchVariance,
    currentState.X(MetaState::ORIENTATION_Z),
    0,                             // orientation covariance is already in state covariance
    currentObservation.levelArm);

  // Compute innovation
  Inn_ = currentObservation.Y();
  Inn_ -= currentState.X().segment<2>(MetaState::POSITION_X);
  Inn_ -= levelArmCompensation_.getPosition().segment<2>(0);

  // Compute innovation covariance
  //  this->R_ = currentObservation.R();
  //  this->R_ += levelArmCompensation_.getPositionCovariance().block<2,2>(0,0);
  H_.template block<2, 1>(0, 2) = levelArmCompensation_.getJacobian().block<2, 1>(0, 2);
  QInn_ = H_ * currentState.P() * H_.transpose() + currentObservation.R();
  QInn_ += levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0);

  // log
  if (logFile_.is_open()) {
    logFile_ << duration.count() << ",";
    logFile_ << currentObservation.Y(0) << ",";
    logFile_ << currentObservation.Y(1) << ",";
    logFile_ << currentObservation.R(0, 0) << ",";
    logFile_ << currentObservation.R(0, 1) << ",";
    logFile_ << currentObservation.R(1, 1) << ",";
    logFile_ << currentState.X(0) << ",";
    logFile_ << currentState.X(1) << ",";
    logFile_ << currentState.X(2) << ",";
    logFile_ << currentState.P(0, 0) << ",";
    logFile_ << currentState.P(0, 1) << ",";
    logFile_ << currentState.P(0, 1) << ",";
    logFile_ << currentState.P(1, 1) << ",";
    logFile_ << currentState.P(1, 2) << ",";
    logFile_ << currentState.P(2, 2) << ",";
    logFile_ << currentAddon.roll << ",";
    logFile_ << currentAddon.pitch << ",";
    logFile_ << levelArmCompensation_.getPosition()(0) << ",";
    logFile_ << levelArmCompensation_.getPosition()(1) << ",";
    logFile_ << levelArmCompensation_.getPosition()(2) << ",";
    logFile_ << currentObservation.Y(0) - Inn_.x() << ",";
    logFile_ << currentObservation.Y(1) - Inn_.y() << ",";
    logFile_ << Inn_.x() << ",";
    logFile_ << Inn_.y() << ",";
  }

  // Update state vector
  bool success = updateState_(currentState);
  if (success) {
    currentAddon.lastExteroceptiveUpdate.time = duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
  }

  if (logFile_.is_open()) {
    logFile_ << mahalanobisDistance_ << ",";
    logFile_ << success << ",\n";
  }

  if (!isPositiveSemiDefiniteMatrix(currentState.P())) {
    std::cout << "X " << std::endl;
    std::cout << previousState.X() << std::endl;
    std::cout << "P " << std::endl;
    std::cout << previousState.P() << std::endl;

    std::cout << "Inn_ " << std::endl;
    std::cout << Inn_ << std::endl;
    std::cout << "QInn_ " << std::endl;
    std::cout << QInn_ << std::endl;
    std::cout << "H_ " << std::endl;
    std::cout << H_ << std::endl;

    std::cout << "Y " << std::endl;
    std::cout << currentObservation.Y() << std::endl;
    std::cout << "R " << std::endl;
    std::cout << currentObservation.R() << std::endl;
    std::cout << "X " << std::endl;
    std::cout << currentState.X() << std::endl;
    std::cout << "P " << std::endl;
    std::cout << currentState.P() << std::endl;
  }
  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}

//-----------------------------------------------------------------------------
bool R2WLocalisationKFUpdaterPosition::set_(
  const Duration & duration,
  const Observation & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddon)
{
  if (!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentState.X(MetaState::ORIENTATION_Z)))
  {
    currentState.X().segment<2>(MetaState::POSITION_X) = currentObservation.Y();

    currentState.P().block<2, 2>(
      MetaState::POSITION_X,
      MetaState::POSITION_X) = currentObservation.R();

    applyLevelArmCompensation(
      currentState,
      currentAddon,
      levelArmCompensation_,
      currentObservation.levelArm);

    currentAddon.lastExteroceptiveUpdate.time = duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
    return true;
  } else {
    return false;
  }
}

}  // namespace romea
