#ifndef romea_R2WLocalisationUpdaterAttitude_hpp
#define romea_R2WLocalisationUpdaterAttitude_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../ObservationAttitude.hpp"
#include "../LocalisationFSMState.hpp"

namespace romea {


template <class MetaState>
class R2WLocalisationUpdaterAttitude
{

public :

  using Observation=ObservationAttitude;

  R2WLocalisationUpdaterAttitude()
  {
  }

  void update(const Duration & /*duration*/,
              const Observation &currentObservation,
              LocalisationFSMState & /*currentFSMState*/,
              MetaState &currentMetaState)
  {
    currentMetaState.addon.roll=currentObservation.Y(ObservationAttitude::ROLL);
    currentMetaState.addon.pitch=currentObservation.Y(ObservationAttitude::PITCH);
    currentMetaState.addon.rollPitchVariance=currentObservation.R(ObservationAttitude::ROLL,
                                                                  ObservationAttitude::ROLL);
  }


};

}



#endif
