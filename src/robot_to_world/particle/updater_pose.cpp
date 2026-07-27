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

// romea
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/robot_to_world/particle/updater_pose.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WPFUpdaterPose::R2WPFUpdaterPose(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const size_t & number_of_particles,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  PFGaussianUpdaterBase(number_of_particles, maximal_mahalanobis_distance),
  cos_courses_(RowMajorVector::Zero(number_of_particles)),
  sin_courses_(RowMajorVector::Zero(number_of_particles)),
  lever_arm_compensation_()
{
}

//--------------------------------------------------------------------------
void R2WPFUpdaterPose::update(
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
        const auto previous_fsm_state = current_fsm_State;
        current_fsm_State = FSMState::RUNNING;
        notify_fsm_event_(
          previous_fsm_state,
          current_fsm_State,
          "INIT DONE (POSE), GO TO RUNNING MODE");
      }
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == trigger_mode::ALWAYS) {
        try {
          update_(
            duration, current_observation, current_meta_state.state, current_meta_state.addon);
        } catch (...) {
          const auto previous_fsm_state = current_fsm_State;
          current_fsm_State = FSMState::INIT;
          current_meta_state.state.reset();
          current_meta_state.addon.reset();
          notify_fsm_event_(
            previous_fsm_state,
            current_fsm_State,
            "FILTER DEGENERESCENCE, RESET AND GO TO INIT MODE");
        }
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPose::update_(
  const Duration & duration,
  Observation current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // compute level arm compensation
  lever_arm_compensation_.compute(
    current_add_on.roll,
    current_add_on.pitch,
    current_add_on.roll_pitch_variance,
    0,
    0,
    current_observation.lever_arm);

  double varxyantenna = lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0).trace();
  const Eigen::Vector3d & antennaPosition = lever_arm_compensation_.getPosition();
  const double & xantenna = antennaPosition(0);
  const double & yantenna = antennaPosition(1);

  // compute apriori observations
  const auto & courses = current_state.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = current_state.particles.row(MetaState::POSITION_X);
  const auto & y = current_state.particles.row(MetaState::POSITION_Y);

  cos_courses_ = courses.array().cos();
  sin_courses_ = courses.array().sin();
  apriori_observations_.row(MetaState::POSITION_X) =
    x + (cos_courses_ * xantenna - sin_courses_ * yantenna);
  apriori_observations_.row(MetaState::POSITION_Y) =
    y + (sin_courses_ * xantenna + cos_courses_ * yantenna);
  apriori_observations_.row(MetaState::ORIENTATION_Z) = courses;

  // update weights and resample
  current_observation.R(MetaState::POSITION_X, MetaState::POSITION_X) += varxyantenna;
  current_observation.R(MetaState::POSITION_Y, MetaState::POSITION_Y) += varxyantenna;
  if (update_state_(current_state, current_observation)) {
    current_add_on.dead_reckoning_tracking.start_time = duration;
    current_add_on.dead_reckoning_tracking.start_travelled_distance = current_add_on.travelled_distance;
  }
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPose::compute_innovation_(
  const PFGaussianUpdaterBase::Observation & observation, const RawMajorVector & weights)
{
  double weight_sum = weights.sum();

  apriori_observation_.Y(MetaState::POSITION_X) =
    (apriori_observations_.row(MetaState::POSITION_X) * weights).sum() / weight_sum;
  apriori_observation_.Y(MetaState::POSITION_Y) =
    (apriori_observations_.row(MetaState::POSITION_Y) * weights).sum() / weight_sum;
  apriori_observation_.Y(MetaState::ORIENTATION_Z) = std::atan2(
    (apriori_observations_.row(MetaState::ORIENTATION_Z).sin() * weights).sum() / weight_sum,
    (apriori_observations_.row(MetaState::ORIENTATION_Z).cos() * weights).sum() / weight_sum);

  for (int i = 0; i < 3; ++i) {
    apriori_mean_centered_observations_.row(i) =
      apriori_observations_.row(i) - apriori_observation_.Y(i);
  }

  // TODO(jean) à vectoriser
  for (size_t n = 0; n < number_of_particles_; n++) {
    apriori_mean_centered_observations_(MetaState::ORIENTATION_Z, n) =
      betweenMinusPiAndPi(apriori_mean_centered_observations_(MetaState::ORIENTATION_Z, n));
  }

  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = i; j < 3; ++j) {
      apriori_observation_.R(i, j) = apriori_observation_.R(j, i) =
        (apriori_mean_centered_observations_.row(i) * apriori_mean_centered_observations_.row(j) *
         weights)
          .sum() /
        weight_sum;
    }
  }

  this->Inn_ = observation.Y() - apriori_observation_.Y();
  this->QInn_ = observation.R() + apriori_observation_.R();
}

//-----------------------------------------------------------------------------
bool R2WPFUpdaterPose::set_(
  const Duration & duration,
  const Observation & current_observation,
  const Input & current_input,
  State & current_state,
  AddOn & current_add_on)
{
  if (
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(current_input.U(MetaState::ANGULAR_SPEED_Z_BODY))) {
    Eigen::Vector3d pose = current_observation.Y();
    Eigen::Matrix3d poseCovariance = current_observation.R();

    lever_arm_compensation_.compute(
      current_add_on.roll,
      current_add_on.pitch,
      current_add_on.roll_pitch_variance,
      current_observation.Y(ObservationPose::ORIENTATION_Z),
      current_observation.R(ObservationPose::ORIENTATION_Z, ObservationPose::ORIENTATION_Z),
      current_observation.lever_arm);

    pose.segment<2>(0) -= lever_arm_compensation_.getPosition().segment<2>(0);
    poseCovariance.block<2, 2>(0, 0) +=
      lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0);

    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(pose, poseCovariance);
    randomGenerator.fill(current_state.particles);

    current_add_on.dead_reckoning_tracking.start_time = duration;
    current_add_on.dead_reckoning_tracking.start_travelled_distance = current_add_on.travelled_distance;

    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
