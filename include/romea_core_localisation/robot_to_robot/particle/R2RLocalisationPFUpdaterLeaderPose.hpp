#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERLEADERPOSE_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERLEADERPOSE_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include "romea_core_localisation/ObservationPose.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFMetaState.hpp"

namespace romea {

class R2RLocalisationPFUpdaterLeaderPose : public LocalisationUpdaterExteroceptive
{
public:
  using Observation = ObservationPose;
  using MetaState = R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;

public :

  R2RLocalisationPFUpdaterLeaderPose(const std::string & updaterName,
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

  bool set_(const Duration & duration,
            const Observation  & currentObservation,
            const Input & currentInput,
            State & currentState,
            AddOn & currentAddOn);
};


}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFUPDATERLEADERPOSE_HPP_





