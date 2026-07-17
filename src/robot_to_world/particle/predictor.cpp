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
#include "romea_core_localisation/robot_to_world/particle/predictor.hpp"

#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WPFPredictor::R2WPFPredictor(
  const Duration & maximal_duration_in_dead_reckoning,
  const double & maximal_travelled_distance_in_dead_reckoning,
  const double & maximal_position_circular_error_probable,
  const size_t & number_of_particles)
: PredictorBase<MetaState>(
    maximal_duration_in_dead_reckoning,
    maximal_travelled_distance_in_dead_reckoning,
    maximal_position_circular_error_probable),
  vxdT_(0),
  vydT_(0),
  cos_courses_(RowMajorVector::Zero(number_of_particles)),
  sin_courses_(RowMajorVector::Zero(number_of_particles)),
  randomU_(RowMajorMatrix::Zero(3, number_of_particles))
{
}

//--------------------------------------------------------------------------
void R2WPFPredictor::predict_(const MetaState & previous_meta_state, MetaState & current_meta_state)
{
  current_meta_state.input = previous_meta_state.input;

  predictState_(previous_meta_state.state, previous_meta_state.input, current_meta_state.state);

  predictAddOn_(previous_meta_state.addon, current_meta_state.addon);

  //  assert(isPositiveSemiDefiniteMatrix(current_meta_state.state.P()));
  assert(isPositiveSemiDefiniteMatrix(current_meta_state.input.QU()));
}

//--------------------------------------------------------------------------
void R2WPFPredictor::drawInputs(const Input & previous_input)
{
  vxdT_ = previous_input.U(MetaState::LINEAR_SPEED_X_BODY) * dt_;
  vydT_ = previous_input.U(MetaState::LINEAR_SPEED_Y_BODY) * dt_;

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(previous_input.U() * dt_, previous_input.QU() * dt_ * dt_);
  randomGenerator.fill(randomU_);
}

//------------------------------------------------------------------------------
void R2WPFPredictor::predictState_(
  const State & previous_state, const Input & previous_input, State & current_state)
{
  drawInputs(previous_input);

  // predict particles
  current_state.particles.row(MetaState::ORIENTATION_Z) =
    previous_state.particles.row(MetaState::ORIENTATION_Z) +
    randomU_.row(MetaState::ANGULAR_SPEED_Z_BODY);

  auto courses = current_state.particles.row(MetaState::ORIENTATION_Z);
  for (int n = 0; n < previous_state.particles.cols(); ++n) {
    courses(n) = between0And2Pi(courses(n));
  }

  cos_courses_ = current_state.particles.row(MetaState::ORIENTATION_Z).cos();
  sin_courses_ = current_state.particles.row(MetaState::ORIENTATION_Z).sin();

  current_state.particles.row(MetaState::POSITION_X) =
    previous_state.particles.row(MetaState::POSITION_X) +
    cos_courses_ * randomU_.row(MetaState::LINEAR_SPEED_X_BODY) -
    sin_courses_ * randomU_.row(MetaState::LINEAR_SPEED_Y_BODY);

  current_state.particles.row(MetaState::POSITION_Y) =
    previous_state.particles.row(MetaState::POSITION_Y) +
    sin_courses_ * randomU_.row(MetaState::LINEAR_SPEED_X_BODY) +
    cos_courses_ * randomU_.row(MetaState::LINEAR_SPEED_Y_BODY);

  current_state.weights = previous_state.weights;
}

//------------------------------------------------------------------------------
void R2WPFPredictor::predictAddOn_(const AddOn & previous_add_on, AddOn & current_add_on)
{
  current_add_on.roll = previous_add_on.roll;
  current_add_on.pitch = previous_add_on.pitch;
  current_add_on.roll = previous_add_on.roll_pitch_variance;
  current_add_on.last_exteroceptive_update = previous_add_on.last_exteroceptive_update;
  current_add_on.travelled_distance =
    previous_add_on.travelled_distance + std::sqrt(vxdT_ * vxdT_ + vydT_ * vydT_);
}

//-----------------------------------------------------------------------------
bool R2WPFPredictor::stop_(const Duration & duration, const MetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration - metaState.addon.last_exteroceptive_update.time;

  double travelledDistanceInDeadReckoningMode =
    metaState.addon.travelled_distance -
    metaState.addon.last_exteroceptive_update.travelled_distance;

  double positionCircularErrorProbability = position_circular_error_probability_(metaState);

  return positionCircularErrorProbability > maximal_position_circular_error_probable_ ||
         travelledDistanceInDeadReckoningMode > maximal_travelled_distance_in_dead_reckoning_ ||
         durationInDeadReckoningMode > maximal_duration_in_dead_reckoning_;
}

//-----------------------------------------------------------------------------
double R2WPFPredictor::position_circular_error_probability_(const MetaState & /*metaState*/) const
{
  return 0;
}

//-----------------------------------------------------------------------------
void R2WPFPredictor::reset_(MetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
