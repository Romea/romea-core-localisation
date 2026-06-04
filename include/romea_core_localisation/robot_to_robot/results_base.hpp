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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__RESULTS_BASE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__RESULTS_BASE_HPP_

// std
#include <utility>

// romea
#include "romea_core_common/geometry/PoseAndTwist3D.hpp"
#include "romea_core_common/time/Time.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class State>
class R2RResultsBase : public State
{
public:
  template<typename... Args>
  R2RResultsBase(Args... args) : State(std::forward<Args>(args)...), duration_(Duration::zero())
  {
  }

  virtual ~R2RResultsBase() = default;

  virtual void set_duration(const Duration & duration) { duration_ = duration; }

  virtual const double & get_leader_x() const = 0;
  virtual const double & get_leader_y() const = 0;
  virtual const double & get_leader_orientation() const = 0;

  virtual Eigen::Vector3d get_leader_pose() const = 0;
  virtual Eigen::Matrix3d get_leader_pose_covariance() const = 0;

  virtual const double & get_linear_speed() const = 0;
  virtual const double & get_lateral_speed() const = 0;
  virtual const double & get_angular_speed() const = 0;

  virtual Eigen::Vector3d get_twist() const = 0;
  virtual Eigen::Matrix3d get_twist_covariance() const = 0;

  virtual const double & get_leader_linear_speed() const = 0;
  virtual const double & get_leader_lateral_speed() const = 0;
  virtual const double & get_leader_angular_speed() const = 0;

  virtual Eigen::Vector3d get_leader_twist() const = 0;
  virtual Eigen::Matrix3d get_leader_twist_covariance() const = 0;

  virtual Pose2D to_leader_pose2d() const = 0;
  virtual PoseAndTwist2D to_leader_pose_and_body_twist2d() const = 0;

protected:
  Duration duration_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__RESULTS_BASE_HPP_
