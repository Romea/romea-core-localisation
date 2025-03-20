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
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & numberOfParticles,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  PFGaussianUpdaterCore(numberOfParticles, maximalMahalanobisDistance),
  leverArms_(RowMajorMatrix::Zero(2, numberOfParticles)),
  cosCourses_(RowMajorVector::Zero(numberOfParticles)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles)),
  lever_arm_compensation_()
{
}

//--------------------------------------------------------------------------
void R2WPFUpdaterPosition::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  switch (currentFSMState) {
    case FSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        std::cout << " FSM : INIT DONE (POSITION + COURSE), GO TO RUNNING MODE " << std::endl;
        currentFSMState = FSMState::RUNNING;
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
          std::cout << " FSM : FILTER DEGENERECENCE, RESET AND GO TO INIT MODE" << std::endl;
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
void R2WPFUpdaterPosition::update_(
  const Duration & duration,
  Observation currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  // compute antenna attitude compensation
  lever_arm_compensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.roll_pitch_variance,
    0,
    0,
    currentObservation.lever_arm);


  double varxyantenna = lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0).trace();
  const Eigen::Vector3d & antennaPosition = lever_arm_compensation_.getPosition();
  const double & xantenna = antennaPosition(0);
  const double & yantenna = antennaPosition(1);

  // compute apriori observations
  const auto & courses = currentState.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = currentState.particles.row(MetaState::POSITION_X);
  const auto & y = currentState.particles.row(MetaState::POSITION_Y);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();
  aprioriObservations_.row(MetaState::POSITION_X) = x +
    (cosCourses_ * xantenna - sinCourses_ * yantenna);
  aprioriObservations_.row(MetaState::POSITION_Y) = y +
    (sinCourses_ * xantenna + cosCourses_ * yantenna);

  // update weights and resample
  currentObservation.R(MetaState::POSITION_X, MetaState::POSITION_X) += varxyantenna;
  currentObservation.R(MetaState::POSITION_Y, MetaState::POSITION_Y) += varxyantenna;
  if (updateState_(currentState, currentObservation)) {
    currentAddon.last_exteroceptive_update.time = duration;
    currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
  }
}

//-----------------------------------------------------------------------------
bool R2WPFUpdaterPosition::set_(
  const Duration & duration,
  Observation const & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddon)
{
  currentAddon.last_exteroceptive_update.time = duration;
  currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;

  if (!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentState.particles(MetaState::ORIENTATION_Z, 0)))
  {
    computeLevelArms_(currentObservation, currentAddon);
    setParticlePositions_(currentObservation, currentState);
    applyLevelArmCompentations_(currentState);
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::computeLevelArms_(
  Observation const & currentObservation,
  const AddOn & currentAddon)
{
  NormalRandomArrayGenerator2D<double> randomGenerator;

  lever_arm_compensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.roll_pitch_variance,
    0,
    0,
    currentObservation.lever_arm);

  randomGenerator.init(
    lever_arm_compensation_.getPosition().segment<2>(0),
    lever_arm_compensation_.getPosition().block<2, 2>(0, 0));

  randomGenerator.fill(leverArms_);
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::setParticlePositions_(
  const Observation & currentObservation,
  State & currentState)
{
  auto particlePositions = currentState.particles.block(2, numberOfParticles_, 0, 0);

  NormalRandomArrayGenerator2D<double> randomGenerator;
  randomGenerator.init(currentObservation.Y(), currentObservation.R());
  randomGenerator.fill(particlePositions);
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterPosition::applyLevelArmCompentations_(State & currentState)
{
  cosCourses_ = currentState.particles.row(MetaState::ORIENTATION_Z).cos();
  sinCourses_ = currentState.particles.row(MetaState::ORIENTATION_Z).sin();

  currentState.particles.row(MetaState::POSITION_X) -=
    cosCourses_ * leverArms_.row(MetaState::POSITION_X) -
    sinCourses_ * leverArms_.row(MetaState::POSITION_Y);

  currentState.particles.row(MetaState::POSITION_Y) -=
    sinCourses_ * leverArms_.row(MetaState::POSITION_X) +
    cosCourses_ * leverArms_.row(MetaState::POSITION_Y);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
