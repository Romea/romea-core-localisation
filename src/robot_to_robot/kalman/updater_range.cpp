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

// Eigen
#include <Eigen/LU>
#include <Eigen/SVD>

// romea
#include <romea_core_common/math/Matrix.hpp>

// std
#include <iostream>
#include <string>

// local
#include "romea_core_localisation/robot_to_robot/kalman/updater_range.hpp"

namespace
{
const double UNSCENTED_TRANSFORM_KAPPA = 3;
const double UNSCENTED_TRANSFORM_ALPHA = 0.75;
const double UNSCENTED_TRANSFORM_BETA = 2;
}  // namespace

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RKFUpdaterRange::R2RKFUpdaterRange(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  UKFUpdaterBase(
    UNSCENTED_TRANSFORM_KAPPA,
    UNSCENTED_TRANSFORM_ALPHA,
    UNSCENTED_TRANSFORM_BETA,
    maximal_mahalanobis_distance)
{
}

//-----------------------------------------------------------------------------
void R2RKFUpdaterRange::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  rate_diagnostic_.evaluate(duration);

  if (current_fsm_State == FSMState::RUNNING) {
    try {
      update_(duration, current_observation, current_meta_state.state, current_meta_state.addon);
    } catch (...) {
      std::cout << " FSM : RANGE UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
      current_meta_state.state.reset();
      current_meta_state.addon.reset();
      current_fsm_State = FSMState::INIT;
    }
  }
}

//-----------------------------------------------------------------------------
void R2RKFUpdaterRange::update_(
  const Duration & duration,
  const Observation & current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // compute sigma points
  compute_state_sigma_points_(current_state);

  // get position of the follower tag
  double ix = current_observation.initiator_position.x();
  double iy = current_observation.initiator_position.y();
  double iz = current_observation.initiator_position.z();

  // get the position of leader tag
  double rx = current_observation.responder_position.x();
  double ry = current_observation.responder_position.y();
  double rz = current_observation.responder_position.z();

  // propagation of the sigma points
  for (size_t n = 0; n < 7; ++n) {
    const double & x = state_sigma_points_[n](0);
    const double & y = state_sigma_points_[n](1);
    const double coso = std::cos(state_sigma_points_[n](2));
    const double sino = std::sin(state_sigma_points_[n](2));

    propagated_sigma_points_[n] = std::sqrt(
      std::pow(x + rx * coso - ry * sino - ix, 2) + std::pow(y + rx * sino + ry * coso - iy, 2) +
      (rz - iz) * (rz - iz));
  }

  // update state
  bool success = update_state_(current_state, current_observation);

  if (logger_) {
    logger_->addEntry("stamp", durationToSecond(duration));
    logger_->addEntry("range", current_observation.Y());
    logger_->addEntry("cov_range", current_observation.R());
    logger_->addEntry("x", current_state.X(0));
    logger_->addEntry("y", current_state.X(1));
    logger_->addEntry("theta", current_state.X(2));
    logger_->addEntry("cov_x", current_state.P(0, 0));
    logger_->addEntry("cov_xy", current_state.P(0, 1));
    logger_->addEntry("cov_xtheta", current_state.P(0, 1));
    logger_->addEntry("cov_y", current_state.P(1, 1));
    logger_->addEntry("cov_ytheta", current_state.P(1, 2));
    logger_->addEntry("cov_theta", current_state.P(2, 2));
    logger_->addEntry("ix", ix);
    logger_->addEntry("iy", iy);
    logger_->addEntry("iz", iz);
    logger_->addEntry("rx", rx);
    logger_->addEntry("ry", ry);
    logger_->addEntry("rz", rz);
  }

  if (success) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;
  }

  // log
  if (logger_) {
    logger_->addEntry("apriori_range", this->propagated_state_.Y());
    logger_->addEntry("cov_apriori_range", this->propagated_state_.R());
    logger_->addEntry("mahalanobis_distance", this->mahalanobis_distance_);
    logger_->addEntry("success", success);
    logger_->writeRow();
  }

  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
