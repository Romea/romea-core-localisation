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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__RESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__RESULTS_HPP_

#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"
#include "romea_core_localisation/robot_to_robot/results_base.hpp"

namespace romea {
namespace core {
namespace localisation {

class R2RPFResults : public R2RResultsBase<R2RPFMetaState> {
 public:
  explicit R2RPFResults(const size_t& number_of_particles);
  virtual ~R2RPFResults() = default;

  const double& get_leader_x() const override;
  const double& get_leader_y() const override;
  const double& get_leader_orientation() const override;

  Eigen::Vector3d get_leader_pose() const override;
  Eigen::Matrix3d get_leader_pose_covariance() const override;

  const double& get_linear_speed() const override;
  const double& get_lateral_speed() const override;
  const double& get_angular_speed() const override;

  Eigen::Vector3d get_twist() const override;
  Eigen::Matrix3d get_twist_covariance() const override;

  const double& get_leader_linear_speed() const override;
  const double& get_leader_lateral_speed() const override;
  const double& get_leader_angular_speed() const override;

  Eigen::Vector3d get_leader_twist() const override;
  Eigen::Matrix3d get_leader_twist_covariance() const override;

  Pose2D to_leader_pose2d() const override;
  PoseAndTwist2D to_leader_pose_and_body_twist2d() const override;

 private:
  void lazy_compute_estimate_() const;
  void lazy_compute_estimate_covariance_() const;

 private:
  mutable double weight_sum_;

  mutable Duration estimate_stamp_;
  mutable Eigen::Vector3d estimate_;

  mutable Duration estimate_covariance_stamp_;
  mutable Eigen::Matrix3d estimate_covariance_;

  mutable R2RPFMetaState::State::RowMajorMatrix mean_centered_particles_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__RESULTS_HPP_
