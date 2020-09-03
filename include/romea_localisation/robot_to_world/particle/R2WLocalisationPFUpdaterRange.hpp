#ifndef romea_R2WLocalisationPFUpdaterRange_hpp
#define romea_R2WLocalisationPFUpdaterRange_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include "R2WLocalisationPFMetaState.hpp"
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>


namespace romea {


class R2WLocalisationPFUpdaterRange : public LocalisationUpdater, public PFGaussianUpdaterCore<double,3,1>
{

public:

  using Observation = ObservationRange;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;

public :

  R2WLocalisationPFUpdaterRange(const size_t & numberOfParticles,
                                const double & maximalMahalanobisDistance,
                                const bool & disableUpdateFunction);

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

  LevelArmCompensation levelArmCompensation_;

  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;

};

}

#endif
