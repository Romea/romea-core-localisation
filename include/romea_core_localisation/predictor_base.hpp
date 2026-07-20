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

#ifndef ROMEA_CORE_LOCALISATION__PREDICTOR_BASE_HPP_
#define ROMEA_CORE_LOCALISATION__PREDICTOR_BASE_HPP_

// romea
#include <romea_core_common/fsm/FSMEventNotifier.hpp>
#include <romea_core_common/log/Logger.hpp>
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/filter/predictor_base.hpp>

// std
#include <iostream>
#include <limits>
#include <memory>
#include <utility>

// local
#include "romea_core_localisation/fsm_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class State>
class PredictorBase : public FilterPredictorBase<State, FSMState, Duration>
{
public:
  PredictorBase(
    const Duration & maximal_duration_in_dead_reckoning,
    const double & maximal_travelled_distance_in_dead_reckoning,
    const double & maximal_position_circular_error_probable);

  virtual ~PredictorBase() = default;

public:
  void register_logger(std::shared_ptr<Logger> logger);

  void register_fsm_event_callback(FSMEventCallback callback);

  virtual void predict(
    const Duration & previous_duration,
    const FSMState & previous_fsm_state,
    const State & previous_state_vector,
    const Duration & currentDuration,
    FSMState & current_fsm_State,
    State & current_state);

protected:
  virtual bool stop_(const Duration & previous_duration, const State & current_state) = 0;

  virtual void predict_(const State & previous_state_vector, State & current_state) = 0;

  virtual void reset_(State & current_state) = 0;

  virtual double position_circular_error_probability_(const State & current_state) const = 0;

  void notify_fsm_event_(
    const FSMState & previous_state,
    const FSMState & current_state,
    const std::string & description);

protected:
  std::shared_ptr<Logger> logger_;
  FSMEventNotifier fsm_event_notifier_;
  Duration maximal_duration_in_dead_reckoning_;
  double maximal_travelled_distance_in_dead_reckoning_;
  double maximal_position_circular_error_probable_;
  double dt_;
};

//-----------------------------------------------------------------------------
template<class State>
PredictorBase<State>::PredictorBase(
  const Duration & maximal_duration_in_dead_reckoning,
  const double & maximal_travelled_distance_in_dead_reckoning,
  const double & maximal_position_circular_error_probable)
: logger_(nullptr),
  fsm_event_notifier_(),
  maximal_duration_in_dead_reckoning_(maximal_duration_in_dead_reckoning),
  maximal_travelled_distance_in_dead_reckoning_(maximal_travelled_distance_in_dead_reckoning),
  maximal_position_circular_error_probable_(maximal_position_circular_error_probable),
  dt_(0)
{
}

//-----------------------------------------------------------------------------
template<class State>
void PredictorBase<State>::register_logger(std::shared_ptr<Logger> logger)
{
  logger_ = std::move(logger);
}

//-----------------------------------------------------------------------------
template<class State>
void PredictorBase<State>::register_fsm_event_callback(FSMEventCallback callback)
{
  fsm_event_notifier_.register_callback(std::move(callback));
}

//-----------------------------------------------------------------------------
template<class State>
void PredictorBase<State>::notify_fsm_event_(
  const FSMState & previous_state, const FSMState & current_state, const std::string & description)
{
  if (previous_state != current_state) {
    fsm_event_notifier_.notify(make_fsm_event(previous_state, current_state, description));
  }
}

//-----------------------------------------------------------------------------
template<class State>
void PredictorBase<State>::predict(
  const Duration & previous_duration,
  const FSMState & previous_fsm_state,
  const State & previous_state,
  const Duration & current_duration,
  FSMState & current_fsm_State,
  State & current_state)
{
  assert(current_duration >= previous_duration);

  current_fsm_State = previous_fsm_state;
  dt_ = durationToSecond(current_duration - previous_duration);

  if (previous_fsm_state == FSMState::RUNNING) {
    if (dt_ > 0) {
      predict_(previous_state, current_state);
    } else {
      current_state = previous_state;
    }

    if (logger_) {
      const auto duration_in_dead_reckoning =
        current_duration - current_state.addon.last_exteroceptive_update.time;

      const auto travelled_distance_in_dead_reckoning =
        current_state.addon.travelled_distance -
        current_state.addon.last_exteroceptive_update.travelled_distance;

      const auto position_circular_error_probability =
        position_circular_error_probability_(current_state);

      logger_->addEntry("stamp", durationToSecond(current_duration));
      logger_->addEntry("dt", dt_);
      logger_->addEntry("pos_cep", position_circular_error_probability);
      logger_->addEntry("dr_distance", travelled_distance_in_dead_reckoning);
      logger_->addEntry("dr_duration", durationToSecond(duration_in_dead_reckoning));
      logger_->writeRow();
    }

    if (stop_(current_duration, current_state)) {
      reset_(current_state);
      current_fsm_State = FSMState::INIT;
      notify_fsm_event_(
        previous_fsm_state, current_fsm_State, "TOO LONG IN DEAD RECKONING, RESET AND GO TO INIT");
    }
  } else {
    current_state = previous_state;
  }

  //  std::cout << "predict current state "<<std::endl;
  //  std::cout << current_state.state.X() <<std::endl;
  //  std::cout << current_state.state.P() <<std::endl;
  //  std::cout << current_state.input.U() <<std::endl;
  //  std::cout << current_state.input.QU() <<std::endl;
  //  std::cout << "fsm state " <<int(current_fsm_State) <<std::endl;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__PREDICTOR_BASE_HPP_
