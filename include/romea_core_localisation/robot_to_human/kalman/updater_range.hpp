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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__UPDATER_RANGE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__UPDATER_RANGE_HPP_

// std
#include <string>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_filtering/filter/kalman/updater/base/extended.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_range.hpp"
#include "romea_core_localisation/robot_to_human/kalman/meta_state.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2HKFUpdaterRange : public UpdaterExteroceptive, public EKFUpdaterBase<double, 2, 1>
{
public:
  using Observation = ObservationRange;
  using MetaState = R2HKFMetaState;
  using State = R2HKFMetaState::State;
  using Input = R2HKFMetaState::Input;
  using AddOn = R2HKFMetaState::AddOn;

public:
  R2HKFUpdaterRange(
    const std::string & updater_name,
    const double & minimal_rate,
    const trigger_mode & trigger_mode,
    const double & maximal_mahalanobis_distance,
    const bool & usedConstraints);

  void update(
    const Duration & duration,
    const Observation & current_observation,
    FSMState & current_fsm_State,
    MetaState & current_meta_state);

  void useConstraints();

private:
  void update_(
    const Duration & duration,
    const Observation & current_observation,
    State & current_state,
    AddOn & current_add_on);

private:
  // Covariance Eigen Vector decomposition
  Eigen::MatrixXd U_;
  Eigen::MatrixXd W_;

  // Modified Grand Schmidt variable
  Eigen::MatrixXd Amgs_;
  Eigen::MatrixXd Tmgs_;
  double Wmgs_;

  // Constraint observation
  Eigen::MatrixXd Dc_;
  Eigen::VectorXd Yc_;
  Eigen::MatrixXd RYc_;

  bool isConstraintsUsed_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__UPDATER_RANGE_HPP_
