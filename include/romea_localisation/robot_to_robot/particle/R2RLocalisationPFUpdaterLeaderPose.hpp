#ifndef romea_R2RLocalisationPFUpdaterLeaderPose_hpp
#define romea_R2RLocalisationPFUpdaterLeaderPose_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../../ObservationPose.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include "R2RLocalisationPFMetaState.hpp"

namespace romea {

class R2RLocalisationPFUpdaterLeaderPose
{
public:

  using Observation = ObservationPose;
  using MetaState = R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;

public :

  R2RLocalisationPFUpdaterLeaderPose();

  void update(const Duration & duration,
              const Observation  & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

private :

  bool set_(const Duration & duration,
            const Observation  & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);

};


}

#endif





