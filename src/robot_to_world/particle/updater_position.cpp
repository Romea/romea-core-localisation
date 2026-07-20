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

// romea
#include "romea_core_localisation/robot_to_world/particle/updater_position.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WPFUpdaterPosition::R2WPFUpdaterPosition(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const size_t & number_of_particles,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  PFGaussianUpdaterBase(number_of_particles, maximal_mahalanobis_distance),
  leverArms_(RowMajorMatrix::Zero(2, number_of_particles)),
  cos_courses_(RowMajorVector::Zero(number_of_particles)),
  sin_courses_(RowMajorVector::Zero(number_of_particles)),
  lever_arm_compensation_()
{
}

//--------------------------------------------------------------------------
void R2WPFUpdaterPosition::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
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
          "INIT DONE (POSITION + COURSE), GO TO RUNNING MODE");
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
void R2WPFUpdaterPosition::update_(
  const Duration & duration,
  Observation current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // compute antenna attitude compensation
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

  // update weights and resample
  current_observation.R(MetaState::POSITION_X, MetaState::POSITION_X) += varxyantenna;
  current_observation.R(MetaState::POSITION_Y, MetaState::POSITION_Y) += varxyantenna;
  if (update_state_(current_state, current_observation)) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;
  }
}

//-----------------------------------------------------------------------------
bool R2WPFUpdaterPosition::set_(
  const Duration & duration,
  const Observation & current_observation,
  const Input & current_input,
  State & current_state,
  AddOn & current_add_on)
{
  current_add_on.last_exteroceptive_update.time = duration;
  current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;

  if (
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(current_input.U(MetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(current_state.particles(MetaState::ORIENTATION_Z, 0))) {
    computelever_arms_(current_observation, current_add_on);
    setParticlePositions_(current_observation, current_state);
    applylever_armCompentations_(current_state);
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::computelever_arms_(
  const Observation & current_observation, const AddOn & current_add_on)
{
  NormalRandomArrayGenerator2D<double> randomGenerator;

  lever_arm_compensation_.compute(
    current_add_on.roll,
    current_add_on.pitch,
    current_add_on.roll_pitch_variance,
    0,
    0,
    current_observation.lever_arm);

  randomGenerator.init(
    lever_arm_compensation_.getPosition().segment<2>(0),
    lever_arm_compensation_.getPosition().block<2, 2>(0, 0));

  randomGenerator.fill(leverArms_);
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::setParticlePositions_(
  const Observation & current_observation, State & current_state)
{
  auto particlePositions = current_state.particles.block(2, number_of_particles_, 0, 0);

  NormalRandomArrayGenerator2D<double> randomGenerator;
  randomGenerator.init(current_observation.Y(), current_observation.R());
  randomGenerator.fill(particlePositions);
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::applylever_armCompentations_(State & current_state)
{
  cos_courses_ = current_state.particles.row(MetaState::ORIENTATION_Z).cos();
  sin_courses_ = current_state.particles.row(MetaState::ORIENTATION_Z).sin();

  current_state.particles.row(MetaState::POSITION_X) -=
    cos_courses_ * leverArms_.row(MetaState::POSITION_X) -
    sin_courses_ * leverArms_.row(MetaState::POSITION_Y);

  current_state.particles.row(MetaState::POSITION_Y) -=
    sin_courses_ * leverArms_.row(MetaState::POSITION_X) +
    cos_courses_ * leverArms_.row(MetaState::POSITION_Y);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
