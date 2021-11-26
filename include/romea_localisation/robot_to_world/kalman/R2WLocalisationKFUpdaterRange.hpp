#ifndef romea_R2WLocalisationKFUpdaterRange_hpp
#define romea_R2WLocalisationKFUpdaterRange_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "R2WLocalisationKFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include <romea_filtering/kalman/UnscentedKalmanFilterUpdaterCore.hpp>

namespace romea {


class R2WLocalisationKFUpdaterRange : public LocalisationUpdaterExteroceptive, public UKFUpdaterCore<double,3,1>
{

public :

  using Observation = ObservationRange;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  void update_(const Duration & duration,
               const Observation &currentObservation,
               State &currentState,
               AddOn &currentAddon);

protected :

  LevelArmCompensation antennaAtitudeCompensation_;

};

}

#endif
