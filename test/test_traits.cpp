// Copyright 2026 INRAE, French National Research Institute for Agriculture,
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

#include <gtest/gtest.h>

#include <type_traits>

#include "romea_core_filtering/filter/type.hpp"
#include "romea_core_localisation/robot_to_robot/traits.hpp"
#include "romea_core_localisation/robot_to_world/traits.hpp"

TEST(TestR2WTraits, exposesKalmanComponentSet)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::KALMAN>;

  static_assert(std::is_class<Traits::Filter>::value, "");
  static_assert(std::is_class<Traits::Predictor>::value, "");
  static_assert(std::is_class<Traits::Results>::value, "");
  static_assert(std::is_class<Traits::UpdaterPosition>::value, "");
  static_assert(std::is_class<Traits::UpdaterCourse>::value, "");
  static_assert(std::is_class<Traits::UpdaterPose>::value, "");
  static_assert(std::is_class<Traits::UpdaterRange>::value, "");
  static_assert(std::is_class<Traits::UpdaterTwist>::value, "");

  SUCCEED();
}

TEST(TestR2WTraits, exposesParticleComponentSet)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::PARTICLE>;

  static_assert(std::is_class<Traits::Filter>::value, "");
  static_assert(std::is_class<Traits::Predictor>::value, "");
  static_assert(std::is_class<Traits::Results>::value, "");
  static_assert(std::is_class<Traits::UpdaterPosition>::value, "");
  static_assert(std::is_class<Traits::UpdaterCourse>::value, "");
  static_assert(std::is_class<Traits::UpdaterPose>::value, "");
  static_assert(std::is_class<Traits::UpdaterRange>::value, "");
  static_assert(std::is_class<Traits::UpdaterTwist>::value, "");

  SUCCEED();
}

TEST(TestR2RTraits, exposesKalmanComponentSet)
{
  using Traits = romea::core::localisation::R2RTraits<romea::core::KALMAN>;

  static_assert(std::is_class<Traits::Filter>::value, "");
  static_assert(std::is_class<Traits::Predictor>::value, "");
  static_assert(std::is_class<Traits::Results>::value, "");
  static_assert(std::is_class<Traits::UpdaterPose>::value, "");
  static_assert(std::is_class<Traits::UpdaterRange>::value, "");
  static_assert(std::is_class<Traits::UpdaterTwist>::value, "");
  static_assert(std::is_class<Traits::UpdaterLeaderTwist>::value, "");

  SUCCEED();
}

TEST(TestR2RTraits, exposesParticleComponentSet)
{
  using Traits = romea::core::localisation::R2RTraits<romea::core::PARTICLE>;

  static_assert(std::is_class<Traits::Filter>::value, "");
  static_assert(std::is_class<Traits::Predictor>::value, "");
  static_assert(std::is_class<Traits::Results>::value, "");
  static_assert(std::is_class<Traits::UpdaterPose>::value, "");
  static_assert(std::is_class<Traits::UpdaterRange>::value, "");
  static_assert(std::is_class<Traits::UpdaterTwist>::value, "");
  static_assert(std::is_class<Traits::UpdaterLeaderTwist>::value, "");

  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
