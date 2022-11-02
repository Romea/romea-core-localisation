#ifndef romea_R2WLocalisationKFUpdaterPosition_hpp
#define romea_R2WLocalisationKFUpdaterPosition_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/Matrix.hpp>

#include "R2WLocalisationKFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationPosition.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include <romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp>

namespace romea {

class R2WLocalisationKFUpdaterPosition : public LocalisationUpdaterExteroceptive, public KFUpdaterCore<double,3,2>
{

public :

  using Observation = ObservationPosition;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterPosition(const std::string & updaterName,
                                   const double & minimalRate,
                                   const TriggerMode & triggerMode,
                                   const double &maximalMahalanobisDistance,
                                   const std::string & logFilename);

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





