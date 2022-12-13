// gtest
#include <gtest/gtest.h>

// romea
#include "test_utils.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"


//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsmStateToString)
{
  EXPECT_STREQ(romea::toString(romea::LocalisationFSMState::ABORTED).c_str(), "ABORTED");
  EXPECT_STREQ(romea::toString(romea::LocalisationFSMState::RUNNING).c_str(), "RUNNING");
  EXPECT_STREQ(romea::toString(romea::LocalisationFSMState::RESET).c_str(), "RESET");
  EXPECT_STREQ(romea::toString(romea::LocalisationFSMState::INIT).c_str(), "INIT");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsmStateToDiagnosticStatus)
{
  EXPECT_EQ(romea::toDiagnosticStatus(romea::LocalisationFSMState::ABORTED),
            romea::DiagnosticStatus::ERROR);
  EXPECT_EQ(romea::toDiagnosticStatus(romea::LocalisationFSMState::RUNNING),
            romea::DiagnosticStatus::OK);
  EXPECT_EQ(romea::toDiagnosticStatus(romea::LocalisationFSMState::RESET),
            romea::DiagnosticStatus::WARN);
  EXPECT_EQ(romea::toDiagnosticStatus(romea::LocalisationFSMState::INIT),
            romea::DiagnosticStatus::WARN);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
