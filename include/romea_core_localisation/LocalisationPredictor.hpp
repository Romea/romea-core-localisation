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


#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONPREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONPREDICTOR_HPP_


// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/FilterPredictor.hpp>

// std
#include <limits>
#include <iostream>

// local
#include "romea_core_localisation/LocalisationFSMState.hpp"


namespace romea
{
namespace core
{

template<class State>
class LocalisationPredictor : public FilterPredictor<State, LocalisationFSMState, Duration>
{
public:
  LocalisationPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable);

  virtual ~LocalisationPredictor() = default;

public:
  virtual void predict(
    const Duration & previousDuration,
    const LocalisationFSMState & previousFSMState,
    const State & previousStateVector,
    const Duration & currentDuration,
    LocalisationFSMState & currentFSMState,
    State & currentState);

protected:
  virtual bool stop_(const Duration & previousDuration, const State & currentState) = 0;

  virtual void predict_(const State & previousStateVector, State & currentState) = 0;

  virtual void reset_(State & currentFSMState) = 0;

protected:
  Duration maximalDurationInDeadReckoning_;
  double maximalTravelledDistanceInDeadReckoning_;
  double maximalPositionCircularErrorProbable_;
  double dt_;
};

//-----------------------------------------------------------------------------
template<class State>
LocalisationPredictor<State>::LocalisationPredictor(
  const Duration & maximalDurationInDeadReckoning,
  const double & maximalTravelledDistanceInDeadReckoning,
  const double & maximalPositionCircularErrorProbable)
: maximalDurationInDeadReckoning_(maximalDurationInDeadReckoning),
  maximalTravelledDistanceInDeadReckoning_(maximalTravelledDistanceInDeadReckoning),
  maximalPositionCircularErrorProbable_(maximalPositionCircularErrorProbable),
  dt_(0)
{
}

//-----------------------------------------------------------------------------
template<class State>
void LocalisationPredictor<State>::predict(
  const Duration & previousDuration,
  const LocalisationFSMState & previousFSMState,
  const State & previousState,
  const Duration & currentduration,
  LocalisationFSMState & currentFSMState,
  State & currentState)
{
  assert(currentduration >= previousDuration);

  currentFSMState = previousFSMState;
  if (previousFSMState == LocalisationFSMState::RUNNING) {
    dt_ = durationToSecond(currentduration - previousDuration);

    if (dt_ > 0) {
      predict_(previousState, currentState);
    } else {
      currentState = previousState;
    }

    if (stop_(currentduration, currentState)) {
      std::cout << "FSM : TOO LONG IN DEAD RECKONING, RESET AND GO TO INIT " << std::endl;
      reset_(currentState);
      currentFSMState = LocalisationFSMState::INIT;
    }
  } else {
    currentState = previousState;
  }

  //  std::cout << "predict current state "<<std::endl;
  //  std::cout << currentState.state.X() <<std::endl;
  //  std::cout << currentState.state.P() <<std::endl;
  //  std::cout << currentState.input.U() <<std::endl;
  //  std::cout << currentState.input.QU() <<std::endl;
  //  std::cout << "fsm state " <<int(currentFSMState) <<std::endl;
}

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONPREDICTOR_HPP_
