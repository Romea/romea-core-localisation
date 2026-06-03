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
#include <romea_core_common/math/Matrix.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/robot_to_world/kalman/updater_range.hpp"

namespace {

const double UNSCENTED_TRANFORM_KAPPA = 3;
const double UNSCENTED_TRANFORM_ALPHA = 0.75;
const double UNSCENTED_TRANFORM_BETA = 2;

}  // namespace

namespace romea {
namespace core {
namespace localisation {

//--------------------------------------------------------------------------
R2WKFUpdaterRange::R2WKFUpdaterRange(const std::string& updater_name,
                                     const double& minimal_rate,
                                     const trigger_mode& trigger_mode,
                                     const double& maximal_mahalanobis_distance,
                                     const std::string& logFilename)
    : UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode,
                           logFilename),
      UKFUpdaterBase(UNSCENTED_TRANFORM_KAPPA, UNSCENTED_TRANFORM_ALPHA,
                     UNSCENTED_TRANFORM_BETA, maximal_mahalanobis_distance),
      lever_arm_compensation_() {
  set_log_file_header_({"stamp",
                        "range",
                        "cov_range",
                        "x",
                        "y",
                        "theta",
                        "cov_x",
                        "cov_xy",
                        "cov_xtheta",
                        "cov_y",
                        "cov_ytheta",
                        "cov_theta",
                        "ix",
                        "iy",
                        "iz",
                        "rx",
                        "ry",
                        "rz",
                        "apriori_range",
                        "cov_apriori_range",
                        "mahalanobis_distance",
                        "sucess"});
}

//-----------------------------------------------------------------------------
void R2WKFUpdaterRange::update(const Duration& duration,
                               const Observation& current_observation,
                               FSMState& current_fsm_State,
                               MetaState& current_meta_state) {
  //  std::cout << " update range " << std::endl;
  rate_diagnostic_.evaluate(duration);

  if (current_fsm_State == FSMState::RUNNING) {
    if (trigger_mode_ == trigger_mode::ALWAYS) {
      try {
        update_(duration, current_observation, current_meta_state.state,
                current_meta_state.addon);
      } catch (...) {
        std::cout << " FSM : RANGE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"
                  << std::endl;
        current_meta_state.state.reset();
        current_meta_state.state.reset();
        current_fsm_State = FSMState::INIT;
      }
    }
  }
}

//--------------------------------------------------------------------------
void R2WKFUpdaterRange::update_(const Duration& duration,
                                const Observation& current_observation,
                                State& current_state, AddOn& current_add_on) {
  // compute antenna attitude compensation
  lever_arm_compensation_.compute(current_add_on.roll, current_add_on.pitch,
                                  current_add_on.roll_pitch_variance, 0, 0,
                                  current_observation.initiator_position);

  const Eigen::Vector3d& tagAntennaPosition =
      lever_arm_compensation_.getPosition();

  const double& ix = tagAntennaPosition.x();
  const double& iy = tagAntennaPosition.y();
  const double& iz =
      tagAntennaPosition.z() + current_observation.terrain_elevation;

  const double& rx = current_observation.responder_position.x();
  const double& ry = current_observation.responder_position.y();
  const double& rz = current_observation.responder_position.z();

  // compute sigma points
  compute_state_sigma_points_(current_state);

  // progation of the sigma points
  for (size_t n = 0; n < 7; ++n) {
    const double& x = state_sigma_points_[n](0);
    const double& y = state_sigma_points_[n](1);
    const double coso = std::cos(state_sigma_points_[n](2));
    const double sino = std::sin(state_sigma_points_[n](2));

    propagated_sigma_points_[n] = std::sqrt(
        std::pow(x + ix * coso - iy * sino - rx, 2) +
        std::pow(y + ix * sino + iy * coso - ry, 2) + (iz - rz) * (iz - rz));
  }

  if (log_file_.is_open()) {
    log_file_ << duration.count() << " ";
    log_file_ << current_observation.Y() << " ";
    log_file_ << current_observation.R() << " ";
    log_file_ << current_state.X(0) << ",";
    log_file_ << current_state.X(1) << ",";
    log_file_ << current_state.X(2) << ",";
    log_file_ << current_state.P(0, 0) << ",";
    log_file_ << current_state.P(0, 1) << ",";
    log_file_ << current_state.P(0, 1) << ",";
    log_file_ << current_state.P(1, 1) << ",";
    log_file_ << current_state.P(1, 2) << ",";
    log_file_ << current_state.P(2, 2) << ",";
    log_file_ << ix << ",";
    log_file_ << iy << ",";
    log_file_ << iz << ",";
    log_file_ << rx << ",";
    log_file_ << ry << ",";
    log_file_ << rz << ",";
  }

  // update state
  bool success = update_state_(current_state, current_observation);

  if (success) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance =
        current_add_on.travelled_distance;
  }

  // log
  if (log_file_.is_open()) {
    log_file_ << propagated_state_.Y() << " ";
    log_file_ << propagated_state_.R() << " ";
    log_file_ << mahalanobis_distance_ << " ";
    log_file_ << success << "/n";
  }

  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
