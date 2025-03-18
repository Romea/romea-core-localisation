// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <string>

// local
#include "romea_core_localisation/robot_to_world/kalman/updater_pose.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WKFUpdaterPose::R2WKFUpdaterPose(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  KFUpdaterCore(maximalMahalanobisDistance),
  levelArmCompensation_()
{
  H_(0, MetaState::POSITION_X) = 1;
  H_(1, MetaState::POSITION_Y) = 1;
  H_(2, MetaState::ORIENTATION_Z) = 1;

  set_log_file_header_(
    {"stamp",
      "x_obs",
      "y_obs",
      "theta_obs",
      "cov_x_obs",
      "cov_xy_obs",
      "cov_xtheta_obs",
      "cov_y_obs",
      "cov_ytheta_obs",
      "cov_theta_obs",
      "x",
      "y",
      "theta",
      "cov_x",
      "cov_xy",
      "cov_xtheta",
      "cov_y",
      "cov_ytheta",
      "cov_theta",
      "lever_arm_x",
      "lever_arm_y"
      "mahalanobis_distance",
      "success"
    });
}

//--------------------------------------------------------------------------
void R2WKFUpdaterPose::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rate_diagnostic_.evaluate(duration);

  switch (currentFSMState) {
    case FSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        currentFSMState = FSMState::RUNNING;
        std::cout << " FSM : INIT DONE (POSE), GO TO RUNNING MODE" << std::endl;
      }
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == TriggerMode::ALWAYS) {
        try {
          update_(
            duration,
            currentObservation,
            currentMetaState.state,
            currentMetaState.addon);
        } catch (...) {
          std::cout << " FSM : POSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
          currentFSMState = FSMState::INIT;
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
void R2WKFUpdaterPose::update_(
  const Duration & duration,
  const Observation & currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  // compute antenna attitude compensation
  levelArmCompensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.roll_pitch_variance,
    currentState.X(MetaState::ORIENTATION_Z),
    0,                             // orientation covariance is already in state covariance
    currentObservation.lever_arm);


  // Compute innovation
  Inn_[0] = currentObservation.Y(ObservationPose::POSITION_X) -
    currentState.X(MetaState::POSITION_X);
  Inn_[1] = currentObservation.Y(ObservationPose::POSITION_Y) -
    currentState.X(MetaState::POSITION_Y);
  Inn_[2] = betweenMinusPiAndPi(
    currentObservation.Y(ObservationPose::ORIENTATION_Z) -
    currentState.X(MetaState::ORIENTATION_Z));
  Inn_.template segment<2>(0) -= levelArmCompensation_.getPosition().segment<2>(0);

  // Compute innovation covariance
  // this->R_ = currentObservation.R();
  // this->R_.template block<2, 2>(0, 0) +=
  //   levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0);
  H_.template block<2, 1>(0, 2) = levelArmCompensation_.getJacobian().block<2, 1>(0, 2);

  QInn_ = H_ * currentState.P() * H_.transpose() + currentObservation.R();
  QInn_.template block<2, 2>(0, 0) +=
    levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0);

  // log
  if (log_file_.is_open()) {
    log_file_ << duration.count() << ",";
    log_file_ << currentObservation.Y(0) << ",";
    log_file_ << currentObservation.Y(1) << ",";
    log_file_ << currentObservation.Y(2) << ",";
    log_file_ << currentObservation.R(0, 0) << ",";
    log_file_ << currentObservation.R(0, 1) << ",";
    log_file_ << currentObservation.R(0, 2) << ",";
    log_file_ << currentObservation.R(1, 1) << ",";
    log_file_ << currentObservation.R(1, 2) << ",";
    log_file_ << currentObservation.R(2, 2) << ",";
    log_file_ << currentState.X(0) << ",";
    log_file_ << currentState.X(1) << ",";
    log_file_ << currentState.X(2) << ",";
    log_file_ << currentState.P(0, 0) << ",";
    log_file_ << currentState.P(0, 1) << ",";
    log_file_ << currentState.P(0, 2) << ",";
    log_file_ << currentState.P(1, 1) << ",";
    log_file_ << currentState.P(1, 2) << ",";
    log_file_ << currentState.P(2, 2) << ",";
    log_file_ << levelArmCompensation_.getPosition()(0) << ",";
    log_file_ << levelArmCompensation_.getPosition()(1) << ",";
  }

  // Update state vector
  if (updateState_(currentState)) {
    currentAddon.last_exteroceptive_update.time = duration;
    currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
  }

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}


//-----------------------------------------------------------------------------
bool R2WKFUpdaterPose::set_(
  const Duration & duration,
  const ObservationPose & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddon)
{
  if (!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)))
  {
    currentState.X() = currentObservation.Y();
    currentState.P() = currentObservation.R();

    applyLevelArmCompensation(
      currentState,
      currentAddon,
      levelArmCompensation_,
      currentObservation.lever_arm);

    currentAddon.last_exteroceptive_update.time = duration;
    currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
