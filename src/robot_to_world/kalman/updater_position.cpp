// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include "romea_core_localisation/robot_to_world/kalman/updater_position.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WKFUpdaterPosition::R2WKFUpdaterPosition(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  EKFUpdaterBase<double, 3, 2>(maximal_mahalanobis_distance),
  lever_arm_compensation_()
{
  this->H_(0, MetaState::POSITION_X) = 1;
  this->H_(1, MetaState::POSITION_Y) = 1;
}

//--------------------------------------------------------------------------
void R2WKFUpdaterPosition::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      if (set_(
            duration,
            current_observation,
            current_meta_state.input,
            current_meta_state.state,
            current_meta_state.addon)) {
        std::cout << " FSM : INIT DONE (POSITION + COURSE), GO TO RUNNING MODE " << std::endl;
        current_fsm_State = FSMState::RUNNING;
      }
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == trigger_mode::ALWAYS) {
        try {
          update_(
            duration, current_observation, current_meta_state.state, current_meta_state.addon);
        } catch (...) {
          std::cout << " FSM : POSITION UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
          current_fsm_State = FSMState::INIT;
          current_meta_state.state.reset();
          current_meta_state.addon.reset();
        }
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
void R2WKFUpdaterPosition::update_(
  const Duration & duration,
  const Observation & current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  State previous_state = current_state;

  // compute antenna attitude compensation
  lever_arm_compensation_.compute(
    current_add_on.roll,
    current_add_on.pitch,
    current_add_on.roll_pitch_variance,
    current_state.X(MetaState::ORIENTATION_Z),
    0,  // orientation covariance is already in state covariance
    current_observation.lever_arm);

  // Compute innovation
  Inn_ = current_observation.Y();
  Inn_ -= current_state.X().segment<2>(MetaState::POSITION_X);
  Inn_ -= lever_arm_compensation_.getPosition().segment<2>(0);

  // Compute innovation covariance
  H_.template block<2, 1>(0, 2) = lever_arm_compensation_.getJacobian().block<2, 1>(0, 2);
  QInn_ = H_ * current_state.P() * H_.transpose() + current_observation.R();
  QInn_ += lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0);

  // log
  if (logger_) {
    logger_->addEntry("stamp", durationToSecond(duration));
    logger_->addEntry("x_obs", current_observation.Y(0));
    logger_->addEntry("y_obs", current_observation.Y(1));
    logger_->addEntry("cov_x_obs", current_observation.R(0, 0));
    logger_->addEntry("cov_y_obs", current_observation.R(0, 1));
    logger_->addEntry("cov_xy_obs", current_observation.R(1, 1));
    logger_->addEntry("x", current_state.X(0));
    logger_->addEntry("y", current_state.X(1));
    logger_->addEntry("theta", current_state.X(2));
    logger_->addEntry("cov_x", current_state.P(0, 0));
    logger_->addEntry("cov_xy", current_state.P(0, 1));
    logger_->addEntry("cov_xtheta", current_state.P(0, 1));
    logger_->addEntry("cov_y", current_state.P(1, 1));
    logger_->addEntry("cov_ytheta", current_state.P(1, 2));
    logger_->addEntry("cov_theta", current_state.P(2, 2));
    logger_->addEntry("roll", current_add_on.roll);
    logger_->addEntry("pitch", current_add_on.pitch);
    logger_->addEntry("lever_arm_x", lever_arm_compensation_.getPosition()(0));
    logger_->addEntry("lever_arm_y", lever_arm_compensation_.getPosition()(1));
    logger_->addEntry("lever_arm_z", lever_arm_compensation_.getPosition()(2));
    logger_->addEntry("x_ant_a_priori", current_observation.Y(0) - Inn_.x());
    logger_->addEntry("y_ant_a_priori", current_observation.Y(1) - Inn_.y());
    logger_->addEntry("inn_x", Inn_.x());
    logger_->addEntry("inn_y", Inn_.y());
  }

  // Update state vector
  bool success = update_state_(current_state);
  if (success) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;
  }

  if (logger_) {
    logger_->addEntry("mahalanobis_distance", mahalanobis_distance_);
    logger_->addEntry("success", success);
    logger_->writeRow();
  }

  if (!isPositiveSemiDefiniteMatrix(current_state.P())) {
    std::cout << "X " << std::endl;
    std::cout << previous_state.X() << std::endl;
    std::cout << "P " << std::endl;
    std::cout << previous_state.P() << std::endl;

    std::cout << "Inn_ " << std::endl;
    std::cout << Inn_ << std::endl;
    std::cout << "QInn_ " << std::endl;
    std::cout << QInn_ << std::endl;
    std::cout << "H_ " << std::endl;
    std::cout << H_ << std::endl;

    std::cout << "Y " << std::endl;
    std::cout << current_observation.Y() << std::endl;
    std::cout << "R " << std::endl;
    std::cout << current_observation.R() << std::endl;
    std::cout << "X " << std::endl;
    std::cout << current_state.X() << std::endl;
    std::cout << "P " << std::endl;
    std::cout << current_state.P() << std::endl;
  }
  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

//-----------------------------------------------------------------------------
bool R2WKFUpdaterPosition::set_(
  const Duration & duration,
  const Observation & current_observation,
  const Input & current_input,
  State & current_state,
  AddOn & current_add_on)
{
  if (
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(current_input.U(MetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(current_state.X(MetaState::ORIENTATION_Z))) {
    current_state.X().segment<2>(MetaState::POSITION_X) = current_observation.Y();

    current_state.P().block<2, 2>(MetaState::POSITION_X, MetaState::POSITION_X) =
      current_observation.R();

    apply_lever_arm_compensation(
      current_state, current_add_on, lever_arm_compensation_, current_observation.lever_arm);

    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;
    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
