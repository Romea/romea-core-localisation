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
#include "romea_core_localisation/robot_to_world/kalman/updater_pose.hpp"

namespace romea {
namespace core {
namespace localisation {

//-----------------------------------------------------------------------------
R2WKFUpdaterPose::R2WKFUpdaterPose(const std::string& updater_name,
                                   const double& minimal_rate,
                                   const trigger_mode& trigger_mode,
                                   const double& maximal_mahalanobis_distance,
                                   const std::string& logFilename)
    : UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode,
                           logFilename),
      EKFUpdaterBase<double, 3, 3>(maximal_mahalanobis_distance),
      lever_arm_compensation_() {
  H_(0, MetaState::POSITION_X) = 1;
  H_(1, MetaState::POSITION_Y) = 1;
  H_(2, MetaState::ORIENTATION_Z) = 1;

  set_log_file_header_({"stamp",
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
                        "success"});
}

//--------------------------------------------------------------------------
void R2WKFUpdaterPose::update(const Duration& duration,
                              const Observation& current_observation,
                              FSMState& current_fsm_State,
                              MetaState& current_meta_state) {
  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      if (set_(duration, current_observation, current_meta_state.input,
               current_meta_state.state, current_meta_state.addon)) {
        current_fsm_State = FSMState::RUNNING;
        std::cout << " FSM : INIT DONE (POSE), GO TO RUNNING MODE" << std::endl;
      }
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == trigger_mode::ALWAYS) {
        try {
          update_(duration, current_observation, current_meta_state.state,
                  current_meta_state.addon);
        } catch (...) {
          std::cout
              << " FSM : POSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"
              << std::endl;
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
void R2WKFUpdaterPose::update_(const Duration& duration,
                               const Observation& current_observation,
                               State& current_state, AddOn& current_add_on) {
  // compute antenna attitude compensation
  lever_arm_compensation_.compute(
      current_add_on.roll, current_add_on.pitch,
      current_add_on.roll_pitch_variance,
      current_state.X(MetaState::ORIENTATION_Z),
      0,  // orientation covariance is already in state covariance
      current_observation.lever_arm);

  // Compute innovation
  Inn_[0] = current_observation.Y(ObservationPose::POSITION_X) -
            current_state.X(MetaState::POSITION_X);
  Inn_[1] = current_observation.Y(ObservationPose::POSITION_Y) -
            current_state.X(MetaState::POSITION_Y);
  Inn_[2] = betweenMinusPiAndPi(
      current_observation.Y(ObservationPose::ORIENTATION_Z) -
      current_state.X(MetaState::ORIENTATION_Z));
  Inn_.template segment<2>(0) -=
      lever_arm_compensation_.getPosition().segment<2>(0);

  // Compute innovation covariance
  // this->R_ = current_observation.R();
  // this->R_.template block<2, 2>(0, 0) +=
  //   lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0);
  H_.template block<2, 1>(0, 2) =
      lever_arm_compensation_.getJacobian().block<2, 1>(0, 2);

  QInn_ = H_ * current_state.P() * H_.transpose() + current_observation.R();
  QInn_.template block<2, 2>(0, 0) +=
      lever_arm_compensation_.getPositionCovariance().block<2, 2>(0, 0);

  // log
  if (log_file_.is_open()) {
    log_file_ << duration.count() << ",";
    log_file_ << current_observation.Y(0) << ",";
    log_file_ << current_observation.Y(1) << ",";
    log_file_ << current_observation.Y(2) << ",";
    log_file_ << current_observation.R(0, 0) << ",";
    log_file_ << current_observation.R(0, 1) << ",";
    log_file_ << current_observation.R(0, 2) << ",";
    log_file_ << current_observation.R(1, 1) << ",";
    log_file_ << current_observation.R(1, 2) << ",";
    log_file_ << current_observation.R(2, 2) << ",";
    log_file_ << current_state.X(0) << ",";
    log_file_ << current_state.X(1) << ",";
    log_file_ << current_state.X(2) << ",";
    log_file_ << current_state.P(0, 0) << ",";
    log_file_ << current_state.P(0, 1) << ",";
    log_file_ << current_state.P(0, 2) << ",";
    log_file_ << current_state.P(1, 1) << ",";
    log_file_ << current_state.P(1, 2) << ",";
    log_file_ << current_state.P(2, 2) << ",";
    log_file_ << lever_arm_compensation_.getPosition()(0) << ",";
    log_file_ << lever_arm_compensation_.getPosition()(1) << ",";
  }

  // Update state vector
  if (update_state_(current_state)) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance =
        current_add_on.travelled_distance;
  }

  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

//-----------------------------------------------------------------------------
bool R2WKFUpdaterPose::set_(const Duration& duration,
                            const ObservationPose& current_observation,
                            const Input& current_input, State& current_state,
                            AddOn& current_add_on) {
  if (!std::isnan(current_input.U(MetaState::LINEAR_SPEED_X_BODY)) &&
      !std::isnan(current_input.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
      !std::isnan(current_input.U(MetaState::ANGULAR_SPEED_Z_BODY))) {
    current_state.X() = current_observation.Y();
    current_state.P() = current_observation.R();

    apply_lever_arm_compensation(current_state, current_add_on,
                                 lever_arm_compensation_,
                                 current_observation.lever_arm);

    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance =
        current_add_on.travelled_distance;
    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
