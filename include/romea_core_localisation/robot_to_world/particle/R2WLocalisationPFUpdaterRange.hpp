#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERRANGE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERRANGE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea {


class R2WLocalisationPFUpdaterRange :
  public LocalisationUpdaterExteroceptive,
  public PFGaussianUpdaterCore<double, 3, 1>
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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERRANGE_HPP_
