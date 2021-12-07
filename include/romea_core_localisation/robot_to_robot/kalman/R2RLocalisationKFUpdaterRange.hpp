#ifndef _romea_R2RLocalisationKFUpdaterRange_HPP_
#define _romea_R2RLocalisationKFUpdaterRange_HPP_


//local
#include <romea_core_common/time/Time.hpp>
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include <romea_core_filtering/kalman/UnscentedKalmanFilterUpdaterCore.hpp>
#include "R2RLocalisationKFMetaState.hpp"


namespace romea {


class R2RLocalisationKFUpdaterRange : public LocalisationUpdaterExteroceptive, public UKFUpdaterCore<double,3,1>
{

public :

  using Observation = ObservationRange;
  using MetaState = R2RLocalisationKFMetaState;
  using State = R2RLocalisationKFMetaState::State;
  using Input = R2RLocalisationKFMetaState::Input;
  using AddOn = R2RLocalisationKFMetaState::AddOn;

public :

  R2RLocalisationKFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentState);
protected :

  void update_(const Duration & duration,
               const Observation & currentObservation,
               State & currentState,
               AddOn & currentAddOn);

};

}//romea
#endif
