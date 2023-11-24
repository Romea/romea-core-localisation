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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFPREDICTOR_HPP
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFPREDICTOR_HPP

// romea
#include "romea_core_localisation/LocalisationPredictor.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea
{
namespace core
{

class R2WLocalisationPFPredictor : public LocalisationPredictor<R2WLocalisationPFMetaState>
{
public:
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2WLocalisationPFMetaState::State::RowMajorMatrix;

public:
  R2WLocalisationPFPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable,
    const size_t & numberOfParticles);

private:
  bool stop_(
    const Duration & duration,
    const MetaState & state)override;

  void predict_(
    const MetaState & previousMetaState,
    MetaState & currentMetaState)override;

  void reset_(MetaState & metaState)override;

  void predictState_(
    const State & previousState,
    const Input & previousInput,
    State & currentState);

  void predictAddOn_(
    const AddOn & previousAddOn,
    AddOn & currentAddOn);

  void drawInputs(const Input & previousInput);

private:
  double vxdT_, vydT_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  RowMajorMatrix randomU_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFPREDICTOR_HPP
