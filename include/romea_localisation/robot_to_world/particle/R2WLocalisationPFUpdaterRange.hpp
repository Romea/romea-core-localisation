#ifndef romea_R2WLocalisationPFUpdaterRange_hpp
#define romea_R2WLocalisationPFUpdaterRange_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

#include "R2WLocalisationPFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"


namespace romea {


class R2WLocalisationPFUpdaterRange : public LocalisationUpdaterExteroceptive, public PFGaussianUpdaterCore<double,3,1>
{

public:

  using Observation = ObservationRange;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;

public :

  R2WLocalisationPFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const size_t & numberOfParticles,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentState);
protected :

  void update_(const Duration & duration,
               Observation currentObservation,
               State &currentState,
               AddOn &currentAddon);

protected :


  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  LevelArmCompensation levelArmCompensation_;

};

}

#endif
