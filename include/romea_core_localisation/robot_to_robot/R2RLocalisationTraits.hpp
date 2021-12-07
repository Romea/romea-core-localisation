#ifndef __R2RLocalisationTraits_HPP__
#define __R2RLocalisationTraits_HPP__


//romea
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include "kalman/R2RLocalisationKFResults.hpp"
#include "kalman/R2RLocalisationKFUpdaterRange.hpp"
#include "R2RLocalisationUpdaterLeaderTwist.hpp"
#include "kalman/R2RLocalisationKFUpdaterLeaderPose.hpp"
#include "kalman/R2RLocalisationKFPredictor.hpp"
#include "kalman/R2RLocalisationKFResults.hpp"
#include "../LocalisationUpdaterTwist.hpp"

#include <romea_core_filtering/particle/ParticleFilter.hpp>
#include "particle/R2RLocalisationPFResults.hpp"
#include "particle/R2RLocalisationPFUpdaterRange.hpp"
#include "R2RLocalisationUpdaterLeaderTwist.hpp"
#include "particle/R2RLocalisationPFUpdaterLeaderPose.hpp"
#include "particle/R2RLocalisationPFPredictor.hpp"
#include "particle/R2RLocalisationPFResults.hpp"
#include "../LocalisationUpdaterTwist.hpp"


namespace romea {


template<FilterType type>
struct R2RLocalisationTraits
{
}
;

template<>
struct R2RLocalisationTraits<KALMAN>
{
  using Filter = KalmanFilter<R2RLocalisationKFMetaState,LocalisationFSMState,Duration>;
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
  using Filter = ParticleFilter<R2RLocalisationPFMetaState,LocalisationFSMState,Duration>;
  using UpdaterLeaderTwist = R2RLocalisationUpdaterLeaderTwist<R2RLocalisationPFMetaState>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2RLocalisationPFMetaState>;
  using UpdaterPose = R2RLocalisationPFUpdaterLeaderPose;
  using UpdaterRange = R2RLocalisationPFUpdaterRange;
  using Predictor = R2RLocalisationPFPredictor;
  using Results = R2RLocalisationPFResults;
};


}

#endif

