#ifndef _romea_R2RLocalisationKFUpdaterLeaderPose_HPP_
#define _romea_R2RLocalisationKFUpdaterLeaderPose_HPP_


//local
#include <romea_core_common/time/Time.hpp>
#include "../../ObservationPose.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include "R2RLocalisationKFMetaState.hpp"

namespace romea {

class R2RLocalisationKFUpdaterLeaderPose : public LocalisationUpdaterExteroceptive
{

public :

  using Observation = ObservationPose;
  using MetaState = R2RLocalisationKFMetaState;
  using State = R2RLocalisationKFMetaState::State;
  using Input = R2RLocalisationKFMetaState::Input;
  using AddOn = R2RLocalisationKFMetaState::AddOn;

public :

  R2RLocalisationKFUpdaterLeaderPose(const std::string & updaterName,
                                     const double & minimalRate,
                                     const TriggerMode & triggerMode,
                                     const double & maximalMahalanobisDistance,
                                     const std::string & logFilename);

  void update(const Duration & duration,
              const Observation &currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);

};

}//romea
#endif
