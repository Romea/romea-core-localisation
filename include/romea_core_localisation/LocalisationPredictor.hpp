#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONPREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONPREDICTOR_HPP_

// std
#include <limits>
#include <iostream>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/FilterPredictor.hpp>
#include "romea_core_localisation/LocalisationFSMState.hpp"


namespace romea
{


template <class State>
class LocalisationPredictor : public FilterPredictor<State, LocalisationFSMState, Duration>
{
public :

  LocalisationPredictor(const Duration &maximalDurationInDeadReckoning,
                        const double &maximalTravelledDistanceInDeadReckoning,
                        const double &maximalPositionCircularErrorProbable);

  virtual ~LocalisationPredictor() = default;

public :

  virtual void predict(const Duration & previousDuration,
                       const LocalisationFSMState & previousFSMState,
                       const State & previousStateVector,
                       const Duration & currentDuration,
                       LocalisationFSMState & currentFSMState,
                       State & currentState);
protected :

  virtual bool stop_(const Duration & previousDuration, const State & currentState) = 0;

  virtual void predict_(const State & previousStateVector, State & currentState) = 0;

  virtual void reset_(State & currentFSMState) = 0;

protected :

  Duration maximalDurationInDeadReckoning_;
  double maximalTravelledDistanceInDeadReckoning_;
  double maximalPositionCircularErrorProbable_;
  double dt_;
};

//-----------------------------------------------------------------------------
template <class State>
LocalisationPredictor<State>::LocalisationPredictor(
  const Duration & maximalDurationInDeadReckoning,
  const double & maximalTravelledDistanceInDeadReckoning,
  const double & maximalPositionCircularErrorProbable):
  maximalDurationInDeadReckoning_(maximalDurationInDeadReckoning),
  maximalTravelledDistanceInDeadReckoning_(maximalTravelledDistanceInDeadReckoning),
  maximalPositionCircularErrorProbable_(maximalPositionCircularErrorProbable),
  dt_(0)
{
}

//-----------------------------------------------------------------------------
template <class State>
void LocalisationPredictor<State>::predict(const Duration & previousDuration,
                                           const LocalisationFSMState & previousFSMState,
                                           const State &previousState,
                                           const Duration &currentduration,
                                           LocalisationFSMState & currentFSMState,
                                           State &currentState)
{
  assert(currentduration >= previousDuration);

  currentFSMState = previousFSMState;
  if (previousFSMState == LocalisationFSMState::RUNNING)
  {
    dt_ = durationToSecond(currentduration-previousDuration);

    if (dt_ > 0)
    {
      predict_(previousState, currentState);
    } else {
      currentState = previousState;
    }

    if (stop_(currentduration, currentState))
    {
      std::cout << "FSM : TOO LONG IN DEAD RECKONING, RESET AND GO TO INIT "<< std::endl;
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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONPREDICTOR_HPP_
