#ifndef romea_R2WLocalisationKFUpdaterCourse_hpp
#define romea_R2WLocalisationKFUpdaterCourse_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../../ObservationCourse.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include <romea_filtering/kalman/KalmanFilterUpdaterCore.hpp>
#include "R2WLocalisationKFMetaState.hpp"

namespace romea {


class R2WLocalisationKFUpdaterCourse : public LocalisationUpdater, public KFUpdaterCore<double,3,1>
{
  
public :

  using Observation = ObservationCourse;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :
  
  R2WLocalisationKFUpdaterCourse(const double & maximalMahalanobisDistance,
                                 const bool & disableUpdateFunction);
  
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
