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

#include <memory>
#include <type_traits>

#include "romea_core_filtering/filter/type.hpp"
#include "romea_core_localisation/dead_reckoning_tracking.hpp"
#include "romea_core_localisation/filter.hpp"
#include "romea_core_localisation/robot_to_human/traits.hpp"
#include "romea_core_localisation/robot_to_robot/traits.hpp"
#include "romea_core_localisation/robot_to_world/traits.hpp"

TEST(TestR2WTraits, exposesKalmanComponentSet)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::KALMAN>;

  static_assert(std::is_class<Traits::Filter>::value);
  static_assert(std::is_class<Traits::Predictor>::value);
  static_assert(std::is_class<Traits::Results>::value);
  static_assert(std::is_class<Traits::UpdaterPosition>::value);
  static_assert(std::is_class<Traits::UpdaterCourse>::value);
  static_assert(std::is_class<Traits::UpdaterPose>::value);
  static_assert(std::is_class<Traits::UpdaterRange>::value);
  static_assert(std::is_class<Traits::UpdaterTwist>::value);

  SUCCEED();
}

TEST(TestR2WTraits, exposesParticleComponentSet)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::PARTICLE>;

  static_assert(std::is_class<Traits::Filter>::value);
  static_assert(std::is_class<Traits::Predictor>::value);
  static_assert(std::is_class<Traits::Results>::value);
  static_assert(std::is_class<Traits::UpdaterPosition>::value);
  static_assert(std::is_class<Traits::UpdaterCourse>::value);
  static_assert(std::is_class<Traits::UpdaterPose>::value);
  static_assert(std::is_class<Traits::UpdaterRange>::value);
  static_assert(std::is_class<Traits::UpdaterTwist>::value);

  SUCCEED();
}

TEST(TestR2RTraits, exposesKalmanComponentSet)
{
  using Traits = romea::core::localisation::R2RTraits<romea::core::KALMAN>;

  static_assert(std::is_class<Traits::Filter>::value);
  static_assert(std::is_class<Traits::Predictor>::value);
  static_assert(std::is_class<Traits::Results>::value);
  static_assert(std::is_class<Traits::UpdaterPose>::value);
  static_assert(std::is_class<Traits::UpdaterRange>::value);
  static_assert(std::is_class<Traits::UpdaterTwist>::value);
  static_assert(std::is_class<Traits::UpdaterLeaderTwist>::value);

  SUCCEED();
}

TEST(TestR2RTraits, exposesParticleComponentSet)
{
  using Traits = romea::core::localisation::R2RTraits<romea::core::PARTICLE>;

  static_assert(std::is_class<Traits::Filter>::value);
  static_assert(std::is_class<Traits::Predictor>::value);
  static_assert(std::is_class<Traits::Results>::value);
  static_assert(std::is_class<Traits::UpdaterPose>::value);
  static_assert(std::is_class<Traits::UpdaterRange>::value);
  static_assert(std::is_class<Traits::UpdaterTwist>::value);
  static_assert(std::is_class<Traits::UpdaterLeaderTwist>::value);

  SUCCEED();
}

TEST(TestR2HTraits, exposesKalmanComponentSet)
{
  using Traits = romea::core::localisation::R2HTraits<romea::core::KALMAN>;

  static_assert(std::is_class<Traits::Filter>::value);
  static_assert(std::is_class<Traits::Predictor>::value);
  static_assert(std::is_class<Traits::Results>::value);
  static_assert(std::is_class<Traits::UpdaterPosition>::value);
  static_assert(std::is_class<Traits::UpdaterRange>::value);
  static_assert(std::is_class<Traits::UpdaterTwist>::value);

  SUCCEED();
}

TEST(TestFilter, instantiatesKalmanFilter)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::KALMAN>;
  using Filter = romea::core::localisation::Filter<romea::core::KALMAN, Traits>;
  using Predictor = typename Traits::Predictor;
  using Updater = typename Traits::UpdaterTwist;

  auto predictor = std::make_unique<Predictor>(
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(10.0),
      2.0));
  Filter filter(10, std::move(predictor));

  auto updater = std::make_unique<Updater>("twist_updater", 10.0);
  auto update_callback = filter.add_updater(std::move(updater));

  ASSERT_TRUE(update_callback.has_value());
  EXPECT_TRUE(filter.initialize());
  EXPECT_FALSE(
    filter.add_updater(
      std::make_unique<Updater>("twist_updater", 10.0)).has_value());

  static_assert(std::is_invocable_v<
    typename decltype(update_callback)::value_type,
    const romea::core::Duration &,
    const Updater::Observation &>);

  SUCCEED();
}

TEST(TestFilter, instantiatesRobotToHumanKalmanFilter)
{
  using Traits = romea::core::localisation::R2HTraits<romea::core::KALMAN>;
  using Filter = romea::core::localisation::Filter<romea::core::KALMAN, Traits>;
  using Predictor = typename Traits::Predictor;
  using Updater = typename Traits::UpdaterTwist;

  auto predictor = std::make_unique<Predictor>(
    0.1,
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(10.0),
      2.0));
  Filter filter(10, std::move(predictor));

  auto updater = std::make_unique<Updater>("twist_updater", 10.0);
  auto update_callback = filter.add_updater(std::move(updater));

  ASSERT_TRUE(update_callback.has_value());
  EXPECT_TRUE(filter.initialize());
  static_assert(std::is_invocable_v<
    typename decltype(update_callback)::value_type,
    const romea::core::Duration &,
    const Updater::Observation &>);

  SUCCEED();
}

TEST(TestFilter, instantiatesParticleFilter)
{
  using Traits = romea::core::localisation::R2WTraits<romea::core::PARTICLE>;
  using Filter = romea::core::localisation::Filter<romea::core::PARTICLE, Traits>;
  using Predictor = typename Traits::Predictor;
  using Updater = typename Traits::UpdaterTwist;

  auto predictor = std::make_unique<Predictor>(
    100,
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(10.0),
      2.0));
  Filter filter(10, 100, std::move(predictor));

  auto updater = std::make_unique<Updater>("twist_updater", 10.0);
  auto update_callback = filter.add_updater(std::move(updater));

  ASSERT_TRUE(update_callback.has_value());
  EXPECT_TRUE(filter.initialize());
  static_assert(std::is_invocable_v<
    typename decltype(update_callback)::value_type,
    const romea::core::Duration &,
    const Updater::Observation &>);

  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
