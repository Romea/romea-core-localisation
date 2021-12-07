#ifndef romea_R2WLocalisationKFUpdaterCourse_hpp
#define romea_R2WLocalisationKFUpdaterCourse_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include "../../ObservationCourse.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include <romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp>
#include "R2WLocalisationKFMetaState.hpp"

namespace romea {


class R2WLocalisationKFUpdaterCourse : public LocalisationUpdaterExteroceptive, public KFUpdaterCore<double,3,1>
{
  
public :

  using Observation = ObservationCourse;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :
  
  R2WLocalisationKFUpdaterCourse(const std::string & updaterName,
                                 const double & minimalRate,
                                 const TriggerMode & triggerMode,
                                 const double & maximalMahalanobisDistance,
                                 const std::string & logFilename);
  
  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
  
private :
  
  void update_(const Duration & duration,
               const Observation &currentObservation,
               State &currentState,
               AddOn &currentAddon);
  
  void set_(const Duration & duration,
            const Observation & currentObservation,
            State &currentState,
            AddOn &currentAddon);
  
};

}

#endif
