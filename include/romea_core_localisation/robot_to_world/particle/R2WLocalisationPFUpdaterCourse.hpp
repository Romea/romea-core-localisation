#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERCOURSE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERCOURSE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/particle/ParticleFilterUpdaterCore.hpp>

#include "romea_core_localisation/ObservationCourse.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea {


class R2WLocalisationPFUpdaterCourse :
 public LocalisationUpdaterExteroceptive,
 public PFUpdaterCore<double, 3, 1>
{
public :

  using Observation = ObservationCourse;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;

public :

  R2WLocalisationPFUpdaterCourse(const std::string & updaterName,
                                 const double & minimalRate,
                                 const TriggerMode & triggerMode,
                                 const size_t & numberOfParticles,
                                 const double & maximalMahalanobisDistance,
                                 const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentState);

private :

  void update_(const Duration & duration,
               const Observation & currentObservation,
               State &currentState,
               AddOn &currentAddon);

  void set_(const Duration & duration,
            const Observation & currentObservation,
            State &currentState,
            AddOn &currentAddon);
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERCOURSE_HPP_
