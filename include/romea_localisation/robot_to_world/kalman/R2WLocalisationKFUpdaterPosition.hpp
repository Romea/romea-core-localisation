#ifndef romea_R2WLocalisationKFUpdaterPosition_hpp
#define romea_R2WLocalisationKFUpdaterPosition_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_common/math/Matrix.hpp>

#include "R2WLocalisationKFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationPosition.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include <romea_filtering/kalman/KalmanFilterUpdaterCore.hpp>

namespace romea {

class R2WLocalisationKFUpdaterPosition : public LocalisationUpdater, public KFUpdaterCore<double,3,2>
{

public :

  using Observation = ObservationPosition;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterPosition(const double &maximalMahalanobisDistance,
                                   const bool &disableUpdateFunction);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

  void update_(const Duration & duration,
               const Observation & currentObservation,
               State &currentState,
               AddOn &currentAddon);

  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input &currentInput,
            State &currentState,
            AddOn &currentAddon);

private :

  LevelArmCompensation levelArmCompensation_;
};

}

#endif





