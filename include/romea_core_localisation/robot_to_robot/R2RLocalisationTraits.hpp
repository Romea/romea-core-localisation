#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONTRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONTRAITS_HPP_


// romea
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include <romea_core_filtering/particle/ParticleFilter.hpp>
#include "romea_core_localisation/LocalisationUpdaterTwist.hpp"
#include "romea_core_localisation/robot_to_robot/R2RLocalisationUpdaterLeaderTwist.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFResults.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFUpdaterRange.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFUpdaterLeaderPose.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFPredictor.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFResults.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFUpdaterRange.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFUpdaterLeaderPose.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFPredictor.hpp"


namespace romea
{


template<FilterType type>
struct R2RLocalisationTraits
{
}
;

template<>
struct R2RLocalisationTraits<KALMAN>
{
  using Filter = KalmanFilter<R2RLocalisationKFMetaState, LocalisationFSMState, Duration>;
  using UpdaterLeaderTwist = R2RLocalisationUpdaterLeaderTwist<R2RLocalisationKFMetaState>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2RLocalisationKFMetaState>;
  using UpdaterPose = R2RLocalisationKFUpdaterLeaderPose;
  using UpdaterRange = R2RLocalisationKFUpdaterRange;
  using Predictor = R2RLocalisationKFPredictor;
  using Results = R2RLocalisationKFResults;
};

template<>
struct R2RLocalisationTraits<PARTICLE>
{
  using Filter = ParticleFilter<R2RLocalisationPFMetaState, LocalisationFSMState, Duration>;
  using UpdaterLeaderTwist = R2RLocalisationUpdaterLeaderTwist<R2RLocalisationPFMetaState>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2RLocalisationPFMetaState>;
  using UpdaterPose = R2RLocalisationPFUpdaterLeaderPose;
  using UpdaterRange = R2RLocalisationPFUpdaterRange;
  using Predictor = R2RLocalisationPFPredictor;
  using Results = R2RLocalisationPFResults;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONTRAITS_HPP_
