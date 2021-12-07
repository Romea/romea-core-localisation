#ifndef romea_R2WLocalisationKFUpdaterPose_hpp
#define romea_R2WLocalisationKFUpdaterPose_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>

#include "R2WLocalisationKFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationPose.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"
#include <romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp>

namespace romea {


class R2WLocalisationKFUpdaterPose : public LocalisationUpdaterExteroceptive, public KFUpdaterCore<double,3,3>
{

public :

  using Observation = ObservationPose;
  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFUpdaterPose(const std::string & updaterName,
                               const double & minimalRate,
                               const TriggerMode & triggerMode,
                               const double &maximalMahalanobisDistance,
                               const std::string & logFilename);

  void update(const Duration & duration,
              const Observation  & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

private :


  void update_(const Duration & duration,
               const Observation  & currentObservation,
               State &currentState,
               AddOn &currentAddon);


  bool set_(const Duration & duration,
            const ObservationPose  & currentObservation,
            const Input &currentInput,
            State &currentState,
            AddOn &currentAddon);

  //  void applyLevelArmCompensation_(R2WLocalisationKFState & currentState,
  //                                  const Eigen::Vector3d & levelArm);

  LevelArmCompensation levelArmCompensation_;
};

}

#endif





