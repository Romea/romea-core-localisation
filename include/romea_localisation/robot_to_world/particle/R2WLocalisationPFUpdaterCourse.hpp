#ifndef romea_R2WLocalisationUpdaterCourse_hpp
#define romea_R2WLocalisationUpdaterCourse_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_filtering/particle/ParticleFilterUpdaterCore.hpp>

#include "R2WLocalisationPFMetaState.hpp"
#include "../../ObservationCourse.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdaterExteroceptive.hpp"

namespace romea {


class R2WLocalisationPFUpdaterCourse : public LocalisationUpdaterExteroceptive, public PFUpdaterCore<double,3,1>
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

}

#endif
