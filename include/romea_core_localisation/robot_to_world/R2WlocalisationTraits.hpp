#ifndef __R2WLocalisationTraits_HPP__
#define __R2WLocalisationTraits_HPP__


//romea
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include "kalman/R2WLocalisationKFResults.hpp"
#include "kalman/R2WLocalisationKFUpdaterCourse.hpp"
#include "kalman/R2WLocalisationKFUpdaterRange.hpp"
#include "kalman/R2WLocalisationKFUpdaterPose.hpp"
#include "kalman/R2WLocalisationKFUpdaterPosition.hpp"
#include "kalman/R2WLocalisationKFPredictor.hpp"

#include <romea_core_filtering/particle/ParticleFilter.hpp>
#include "particle/R2WLocalisationPFResults.hpp"
#include "particle/R2WLocalisationPFUpdaterCourse.hpp"
#include "particle/R2WLocalisationPFUpdaterRange.hpp"
#include "particle/R2WLocalisationPFUpdaterPose.hpp"
#include "particle/R2WLocalisationPFUpdaterPosition.hpp"
#include "particle/R2WLocalisationPFPredictor.hpp"

#include "R2WLocalisationUpdaterAttitude.hpp"
#include "../LocalisationUpdaterLinearSpeed.hpp"
#include "../LocalisationUpdaterLinearSpeeds.hpp"
#include "../LocalisationUpdaterAngularSpeed.hpp"


namespace romea {

template<FilterType type>
struct R2WLocalisationTraits
{
}
;

template<>
struct R2WLocalisationTraits<KALMAN>
{
  using Filter = KalmanFilter<R2WLocalisationKFMetaState,LocalisationFSMState,Duration>;
  using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2WLocalisationKFMetaState>;
  using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2WLocalisationKFMetaState>;
  using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2WLocalisationKFMetaState>;
  using UpdaterAttitude = R2WLocalisationUpdaterAttitude<R2WLocalisationKFMetaState>;
  using UpdaterCourse = R2WLocalisationKFUpdaterCourse;
  using UpdaterPose = R2WLocalisationKFUpdaterPose;
  using UpdaterPosition = R2WLocalisationKFUpdaterPosition;
  using UpdaterRange = R2WLocalisationKFUpdaterRange;
  using Predictor = R2WLocalisationKFPredictor;
  using Results = R2WLocalisationKFResults;
};

template<>
struct R2WLocalisationTraits<PARTICLE>
{
  using Filter = ParticleFilter<R2WLocalisationPFMetaState,LocalisationFSMState,Duration>;
  using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2WLocalisationPFMetaState>;
  using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2WLocalisationPFMetaState>;
  using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2WLocalisationPFMetaState>;
  using UpdaterAttitude = R2WLocalisationUpdaterAttitude<R2WLocalisationPFMetaState>;
  using UpdaterCourse = R2WLocalisationPFUpdaterCourse;
  using UpdaterPose = R2WLocalisationPFUpdaterPose;
  using UpdaterPosition = R2WLocalisationPFUpdaterPosition;
  using UpdaterRange = R2WLocalisationPFUpdaterRange;
  using Predictor = R2WLocalisationPFPredictor;
  using Results = R2WLocalisationPFResults;
};


}

#endif

