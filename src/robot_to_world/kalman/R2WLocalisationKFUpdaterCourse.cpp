//romea
#include "romea_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterCourse.hpp"
#include <romea_common/math/EulerAngles.hpp>

#include <iostream>

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationKFUpdaterCourse::R2WLocalisationKFUpdaterCourse(const double &maximalMahalanobisDistance,
                                                               const bool & disableUpdateFunction,
                                                               const std::string & logFilename):
  LocalisationUpdater(logFilename,disableUpdateFunction),
  KFUpdaterCore(maximalMahalanobisDistance)
{
  this->H_(0, MetaState::ORIENTATION_Z)=1;
  setLogFileHeader_({"stamp",
                     "course",
                     "cov_course",
                     "theta",
                     "cov_theta"
                    });
}

//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterCourse::update(const Duration & duration,
                                            const Observation &currentObservation,
                                            LocalisationFSMState & currentFSMState,
                                            MetaState & currentMetaState)
{
  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    set_(duration,
         currentObservation,
         currentMetaState.state,
         currentMetaState.addon);
    break;
  case LocalisationFSMState::RUNNING:
    if(duration<updateStartDisableTime_)
    {
      try
      {
        update_(duration,
                currentObservation,
                currentMetaState.state,
                currentMetaState.addon);
      }
      catch(...)
      {
        std::cout << " FSM : COURSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"<< std::endl;
        currentFSMState = LocalisationFSMState::INIT;
        currentMetaState.state.reset();
        currentMetaState.addon.reset();
      }
    }
    break;
  default:
    break;
  }
}
//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterCourse::update_(const Duration & duration,
                                             const Observation & currentObservation,
                                             State & currentState,
                                             AddOn & currentAddon)
{

  Inn_ = betweenMinusPiAndPi(currentObservation.Y()-currentState.X(MetaState::ORIENTATION_Z));

  this->QInn_  = currentObservation.R() + currentState.P(MetaState::ORIENTATION_Z,
                                                         MetaState::ORIENTATION_Z);

  //log
  if(logFile_.is_open())
  {
    logFile_<< duration.count()<<",";
    logFile_<< currentObservation.Y() <<",";
    logFile_<< currentObservation.R() <<",";
    logFile_<< currentState.X(2) <<",";
    logFile_<< currentState.P(2,2) <<",/n";
  }
  //  if(!updateClassic_(currentState)){
  //    //TODO renvoyer un throw
  //  }

  currentAddon.lastExteroceptiveUpdate.time=duration;
  currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;

}

//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterCourse::set_(const Duration & duration,
                                          const Observation & currentObservation,
                                          State & currentState,
                                          AddOn & currentAddon)
{
  currentState.X(MetaState::ORIENTATION_Z)=currentObservation.Y();

  currentState.P(MetaState::ORIENTATION_Z,
                 MetaState::ORIENTATION_Z)=currentObservation.R();


  currentAddon.lastExteroceptiveUpdate.time=duration;
  currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;

}

}
