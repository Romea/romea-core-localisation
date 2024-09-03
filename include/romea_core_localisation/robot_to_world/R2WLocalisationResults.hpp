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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONRESULTS_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>

namespace romea
{
namespace core
{

template<class State>
class R2WLocalisationResults : public State
{

public:
  template<typename ... Args>
  R2WLocalisationResults(Args... args)
  : State(std::forward<Args>(args)...),
    duration_(Duration::zero())
  {
  }

  virtual ~R2WLocalisationResults() = default;

  virtual void setDuration(const Duration & duration)
  {
    duration_ = duration;
  }

  virtual const double & getX() const = 0;
  virtual const double & getY() const = 0;
  virtual const double & getYaw() const = 0;
  virtual const double & getYawVariance() const = 0;

  virtual Eigen::Vector3d getPose() const = 0;
  virtual Eigen::Matrix3d getPoseCovariance() const = 0;

  virtual const double & getLinearSpeed() const = 0;
  virtual const double & getLateralSpeed() const = 0;
  virtual const double & getAngularSpeed() const = 0;

  virtual Eigen::Vector3d getTwist() const = 0;
  virtual Eigen::Matrix3d getTwistCovariance() const = 0;

  virtual Pose2D toPose2D() const = 0;
  virtual Pose3D toPose3D() const = 0;

  virtual PoseAndTwist2D toPoseAndBodyTwist2D() const = 0;
  virtual PoseAndTwist3D toPoseAndBodyTwist3D() const = 0;

protected:
  Duration duration_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONRESULTS_HPP_
