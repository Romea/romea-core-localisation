#ifndef romea_R2WLocalisationPFUpdaterPose_hpp
#define romea_R2WLocalisationPFUpdaterPose_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

#include "R2WLocalisationPFMetaState.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "../../ObservationPose.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"


namespace romea {

class R2WLocalisationPFUpdaterPose :  public LocalisationUpdaterExteroceptive, public PFGaussianUpdaterCore<double,3,3>
{

public :

  using Observation = ObservationPose;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;

public :

  R2WLocalisationPFUpdaterPose(const std::string & updaterName,
                               const double & minimalRate,
                               const TriggerMode & triggerMode,
                               const size_t &numberOfParticles,
                               const double &maximalMahalanobisDistance,
                               const std::string & logFilename);

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

  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  LevelArmCompensation levelArmCompensation_;
};


}

#endif





