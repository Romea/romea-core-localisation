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

// gtest
#include <gtest/gtest.h>

// romea
#include "romea_core_common/fsm/FSMEventNotifier.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/update_monitoring.hpp"
#include "romea_core_localisation/updater_base.hpp"
#include "test_utils.hpp"

using namespace romea::core::localisation;  // NOLINT

class TestUpdaterBase : public UpdaterBase
{
public:
  TestUpdaterBase()
  : UpdaterBase("test_updater", 1.0, Updatertrigger_mode::ALWAYS)
  {
  }

  using UpdaterBase::notify_fsm_event_;
};

struct TestPredictorState
{
  struct Addon
  {
    UpdateMonitoring last_exteroceptive_update;
    double travelled_distance = 0.;
  };

  Addon addon;
};

class TestPredictorBase : public PredictorBase<TestPredictorState>
{
public:
  TestPredictorBase()
  : PredictorBase<TestPredictorState>(romea::core::Duration::max(), 0., 0.)
  {
  }

  using PredictorBase<TestPredictorState>::notify_fsm_event_;

protected:
  bool stop_(const romea::core::Duration &, const TestPredictorState &) override { return false; }

  void predict_(const TestPredictorState & previous_state, TestPredictorState & current_state) override
  {
    current_state = previous_state;
  }

  void reset_(TestPredictorState & current_state) override { current_state = TestPredictorState(); }

  double position_circular_error_probability_(const TestPredictorState &) const override { return 0.; }
};

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsm_stateToString)
{
  EXPECT_STREQ(to_string(FSMState::ABORTED).c_str(), "ABORTED");
  EXPECT_STREQ(to_string(FSMState::RUNNING).c_str(), "RUNNING");
  EXPECT_STREQ(to_string(FSMState::RESET).c_str(), "RESET");
  EXPECT_STREQ(to_string(FSMState::INIT).c_str(), "INIT");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsm_stateToDiagnosticStatus)
{
  EXPECT_EQ(to_diagnostic_status(FSMState::ABORTED), romea::core::DiagnosticStatus::ERROR);
  EXPECT_EQ(to_diagnostic_status(FSMState::RUNNING), romea::core::DiagnosticStatus::OK);
  EXPECT_EQ(to_diagnostic_status(FSMState::RESET), romea::core::DiagnosticStatus::WARN);
  EXPECT_EQ(to_diagnostic_status(FSMState::INIT), romea::core::DiagnosticStatus::WARN);
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkfsm_stateToCommonFSMState)
{
  const auto init_state = to_common_fsm_state(FSMState::INIT);
  EXPECT_STREQ(init_state.name.c_str(), "INIT");
  EXPECT_EQ(init_state.id, 0);

  const auto running_state = to_common_fsm_state(FSMState::RUNNING);
  EXPECT_STREQ(running_state.name.c_str(), "RUNNING");
  EXPECT_EQ(running_state.id, 1);

  const auto reset_state = to_common_fsm_state(FSMState::RESET);
  EXPECT_STREQ(reset_state.name.c_str(), "RESET");
  EXPECT_EQ(reset_state.id, 2);

  const auto aborted_state = to_common_fsm_state(FSMState::ABORTED);
  EXPECT_STREQ(aborted_state.name.c_str(), "ABORTED");
  EXPECT_EQ(aborted_state.id, 3);
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkMakeFSMEvent)
{
  const auto event = make_fsm_event(
    FSMState::INIT, FSMState::RUNNING, "INIT DONE, GO TO RUNNING MODE");

  EXPECT_STREQ(event.previous_state.name.c_str(), "INIT");
  EXPECT_EQ(event.previous_state.id, 0);
  EXPECT_STREQ(event.current_state.name.c_str(), "RUNNING");
  EXPECT_EQ(event.current_state.id, 1);
  EXPECT_STREQ(event.description.c_str(), "INIT DONE, GO TO RUNNING MODE");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkFSMEventNotifier)
{
  romea::core::FSMEventNotifier notifier;
  const auto event = make_fsm_event(FSMState::RUNNING, FSMState::INIT, "RESET");

  EXPECT_NO_THROW(notifier.notify(event));

  bool callback_called = false;
  romea::core::FSMEvent received_event;
  notifier.register_callback(
    [&callback_called, &received_event](const romea::core::FSMEvent & notified_event) {
      callback_called = true;
      received_event = notified_event;
    });

  notifier.notify(event);

  EXPECT_TRUE(callback_called);
  EXPECT_STREQ(received_event.previous_state.name.c_str(), "RUNNING");
  EXPECT_STREQ(received_event.current_state.name.c_str(), "INIT");
  EXPECT_STREQ(received_event.description.c_str(), "RESET");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkUpdaterBaseFSMEventNotification)
{
  TestUpdaterBase updater;
  bool callback_called = false;
  romea::core::FSMEvent received_event;

  updater.register_fsm_event_callback(
    [&callback_called, &received_event](const romea::core::FSMEvent & event) {
      callback_called = true;
      received_event = event;
    });

  updater.notify_fsm_event_(FSMState::INIT, FSMState::RUNNING, "INIT DONE");

  EXPECT_TRUE(callback_called);
  EXPECT_STREQ(received_event.previous_state.name.c_str(), "INIT");
  EXPECT_STREQ(received_event.current_state.name.c_str(), "RUNNING");
  EXPECT_STREQ(received_event.description.c_str(), "INIT DONE");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkUpdaterBaseDoesNotNotifyUnchangedFSMState)
{
  TestUpdaterBase updater;
  bool callback_called = false;

  updater.register_fsm_event_callback(
    [&callback_called](const romea::core::FSMEvent &) { callback_called = true; });

  updater.notify_fsm_event_(FSMState::RUNNING, FSMState::RUNNING, "UNCHANGED");

  EXPECT_FALSE(callback_called);
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkPredictorBaseFSMEventNotification)
{
  TestPredictorBase predictor;
  bool callback_called = false;
  romea::core::FSMEvent received_event;

  predictor.register_fsm_event_callback(
    [&callback_called, &received_event](const romea::core::FSMEvent & event) {
      callback_called = true;
      received_event = event;
    });

  predictor.notify_fsm_event_(FSMState::RUNNING, FSMState::INIT, "DEAD RECKONING LIMIT");

  EXPECT_TRUE(callback_called);
  EXPECT_STREQ(received_event.previous_state.name.c_str(), "RUNNING");
  EXPECT_STREQ(received_event.current_state.name.c_str(), "INIT");
  EXPECT_STREQ(received_event.description.c_str(), "DEAD RECKONING LIMIT");
}

//-----------------------------------------------------------------------------
TEST(TestFSMStateConversion, checkPredictorBaseDoesNotNotifyUnchangedFSMState)
{
  TestPredictorBase predictor;
  bool callback_called = false;

  predictor.register_fsm_event_callback(
    [&callback_called](const romea::core::FSMEvent &) { callback_called = true; });

  predictor.notify_fsm_event_(FSMState::INIT, FSMState::INIT, "UNCHANGED");

  EXPECT_FALSE(callback_called);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
