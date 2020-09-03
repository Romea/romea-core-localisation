#ifndef _romea_R2HLocalisationKFUpdaterLeaderPosition_HPP_
#define _romea_R2HLocalisationKFUpdaterLeaderPosition_HPP_


//romea
#include <romea_common/time/Time.hpp>
#include "../../ObservationPosition.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include "R2HLocalisationKFMetaState.hpp"

namespace romea {


class R2HLocalisationKFUpdaterLeaderPosition
{
public :

  using Observation = ObservationPosition;
  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public :

  R2HLocalisationKFUpdaterLeaderPosition();

  void update(const Duration & duration,
              const Observation &currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);


  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);

};

}//romea
#endif
