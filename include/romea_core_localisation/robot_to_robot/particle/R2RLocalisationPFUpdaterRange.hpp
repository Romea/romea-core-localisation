#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERRANGE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERRANGE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>
#include <romea_core_filtering/particle/ParticleFilterResampling.hpp>
#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFMetaState.hpp"


namespace romea {

class R2RLocalisationPFUpdaterRange :
  public LocalisationUpdaterExteroceptive,
  public PFGaussianUpdaterCore<double, 3, 1>
{
public:
  using Observation = ObservationRange;
  using MetaState = R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2RLocalisationPFMetaState::State::RowMajorVector;

public :

  R2RLocalisationPFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const size_t &numberOfParticles,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  void update_(const Duration & duration,
               const Observation &currentObservation,
               State & currentState,
               AddOn & currentAddOn);

protected :

  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERRANGE_HPP_
