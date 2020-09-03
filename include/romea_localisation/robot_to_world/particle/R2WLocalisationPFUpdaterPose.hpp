#ifndef romea_R2WLocalisationPFUpdaterPose_hpp
#define romea_R2WLocalisationPFUpdaterPose_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../../ObservationPose.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "R2WLocalisationPFMetaState.hpp"
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

namespace romea {

class R2WLocalisationPFUpdaterPose :  public LocalisationUpdater, public PFGaussianUpdaterCore<double,3,3>
{

public :

  using Observation = ObservationPose;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;

public :

  R2WLocalisationPFUpdaterPose(const size_t &numberOfParticles,
                               const double &maximalMahalanobisDistance,
                               const bool &disableUpdateFunction);

  void update(const Duration & duration,
              const Observation  & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

private :

  virtual void computeInnovation_(const PFGaussianUpdaterCore::Observation & observation,
                                  const RawMajorVector & weights)override;

  void update_(const Duration & duration,
               Observation currentObservation,
               State &currentState,
               AddOn &currentAddon);


  bool set_(const Duration & duration,
            const Observation  & currentObservation,
            const Input & currentInput,
            State &currentState,
            AddOn &currentAddon);

private :

  LevelArmCompensation levelArmCompensation_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
};


}

#endif





