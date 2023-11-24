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


// gtest
#include <gtest/gtest.h>

// romea
#include "test_utils.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"


//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsmStateToString)
{
  EXPECT_STREQ(
    romea::core::toString(romea::core::LocalisationFSMState::ABORTED).c_str(), "ABORTED");
  EXPECT_STREQ(
    romea::core::toString(romea::core::LocalisationFSMState::RUNNING).c_str(), "RUNNING");
  EXPECT_STREQ(
    romea::core::toString(romea::core::LocalisationFSMState::RESET).c_str(), "RESET");
  EXPECT_STREQ(
    romea::core::toString(romea::core::LocalisationFSMState::INIT).c_str(), "INIT");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsmStateToDiagnosticStatus)
{
  EXPECT_EQ(
    romea::core::toDiagnosticStatus(romea::core::LocalisationFSMState::ABORTED),
    romea::core::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    romea::core::toDiagnosticStatus(romea::core::LocalisationFSMState::RUNNING),
    romea::core::DiagnosticStatus::OK);
  EXPECT_EQ(
    romea::core::toDiagnosticStatus(romea::core::LocalisationFSMState::RESET),
    romea::core::DiagnosticStatus::WARN);
  EXPECT_EQ(
    romea::core::toDiagnosticStatus(romea::core::LocalisationFSMState::INIT),
    romea::core::DiagnosticStatus::WARN);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
