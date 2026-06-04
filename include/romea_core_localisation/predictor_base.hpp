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
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/filter/predictor_base.hpp>

// std
#include <iostream>
#include <limits>

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

protected:
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
: maximal_duration_in_dead_reckoning_(maximal_duration_in_dead_reckoning),
  maximal_travelled_distance_in_dead_reckoning_(maximal_travelled_distance_in_dead_reckoning),
  maximal_position_circular_error_probable_(maximal_position_circular_error_probable),
  dt_(0)
{
}

//-----------------------------------------------------------------------------
template<class State>
void PredictorBase<State>::predict(
  const Duration & previous_duration,
  const FSMState & previous_fsm_state,
  const State & previous_state,
  const Duration & currentduration,
  FSMState & current_fsm_State,
  State & current_state)
{
  assert(currentduration >= previous_duration);

  current_fsm_State = previous_fsm_state;
  if (previous_fsm_state == FSMState::RUNNING) {
    dt_ = durationToSecond(currentduration - previous_duration);

    if (dt_ > 0) {
      predict_(previous_state, current_state);
    } else {
      current_state = previous_state;
    }

    if (stop_(currentduration, current_state)) {
      std::cout << "FSM : TOO LONG IN DEAD RECKONING, RESET AND GO TO INIT " << std::endl;
      reset_(current_state);
      current_fsm_State = FSMState::INIT;
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
