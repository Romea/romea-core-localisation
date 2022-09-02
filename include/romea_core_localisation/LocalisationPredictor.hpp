#ifndef romea_LocalisationPredictor_hpp
#define romea_LocalisationPredictor_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/FilterPredictor.hpp>
#include "LocalisationFSMState.hpp"
#include "LocalisationStoppingCriteria.hpp"

//std
#include <limits>
#include <iostream>

namespace romea
{


template <class State>
class LocalisationPredictor : public FilterPredictor<State,LocalisationFSMState,Duration>
{

public :

  LocalisationPredictor(const LocalisationStoppingCriteria & stoppingCriteria);

  virtual ~LocalisationPredictor()=default;

public :

  virtual void predict(const Duration & previousDuration,
                       const LocalisationFSMState & previousFSMState,
                       const State & previousStateVector,
                       const Duration & currentDuration,
                       LocalisationFSMState & currentFSMState,
                       State & currentState);
protected :

  virtual bool stop_(const Duration & previousDuration,const State & currentState)=0;

  virtual void predict_(const State & previousStateVector,State & currentState)=0;

  virtual void reset_( State & currentFSMState)=0;

protected :

  LocalisationStoppingCriteria stoppingCriteria_;
  double dt_;
};




//-----------------------------------------------------------------------------
template <class State>
LocalisationPredictor<State>::LocalisationPredictor(const LocalisationStoppingCriteria & stoppingCriteria):
  stoppingCriteria_(stoppingCriteria),
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
  assert(currentduration>=previousDuration);

  currentFSMState = previousFSMState;
  if(previousFSMState == LocalisationFSMState::RUNNING)
  {
    dt_ =durationToSecond(currentduration-previousDuration);

    if(dt_>0)
    {
      predict_(previousState,currentState);
    }
    else
    {
      currentState=previousState;
    }

    if(stop_(currentduration,currentState))
    {
      std::cout << "FSM : TOO LONG IN DEAD RECKONING, RESET AND GO TO INIT "<< std::endl;
      reset_(currentState);
      currentFSMState=LocalisationFSMState::INIT;
    }
  }
  else
  {
    currentState = previousState;

  }

//  std::cout << "predict current state "<<std::endl;
//  std::cout << currentState.state.X() <<std::endl;
//  std::cout << currentState.state.P() <<std::endl;
//  std::cout << currentState.input.U() <<std::endl;
//  std::cout << currentState.input.QU() <<std::endl;
//  std::cout << "fsm state " <<int(currentFSMState) <<std::endl;

}

}

#endif
