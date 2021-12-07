//romea
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterCourse.hpp"
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_common/math/EulerAngles.hpp>

//std
#include <random>

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationPFUpdaterCourse::R2WLocalisationPFUpdaterCourse(const std::string & updaterName, const double & minimalRate,
                                                               const TriggerMode & triggerMode,
                                                               const size_t & numberOfParticles,
                                                               const double & /*maximalMahalanobisDistance*/,
                                                               const std::string & logFilename):
  LocalisationUpdaterExteroceptive(updaterName,
                                   minimalRate,
                                   triggerMode,
                                   logFilename),
  PFUpdaterCore(numberOfParticles)
{

}


//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterCourse::update(const Duration & duration,
                                            const Observation & currentObservation,
                                            LocalisationFSMState & currentFSMState,
                                            MetaState & currentMetaState)
{
  rateDiagnostic_.evaluate(duration);

  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    set_(duration,
         currentObservation,
         currentMetaState.state,
         currentMetaState.addon);
    break;
  case LocalisationFSMState::RUNNING:
    if(triggerMode_==TriggerMode::ALWAYS)
    {
      update_(duration,
              currentObservation,
              currentMetaState.state,
              currentMetaState.addon);
    }
    break;
  default:
    break;
  }
}

//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterCourse::update_(const Duration & duration,
                                             const Observation & currentObservation,
                                             State &currentState,
                                             AddOn &currentAddon)
{

  double course = currentObservation.Y();
  double vonMisesConcentration = 0.5/(1-std::exp(-currentObservation.R()/2));

  const auto & courses=currentState.particles.row(MetaState::ANGULAR_SPEED_Z_BODY);
  currentState.weights*=((courses-course).cos()*vonMisesConcentration).exp();

  currentAddon.lastExteroceptiveUpdate.time=duration;
  currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;
}

//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterCourse::set_(const Duration & duration,
                                          const Observation &currentObservation,
                                          State &currentState,
                                          AddOn &currentAddon)
{

  auto particleCourses = currentState.particles.row(MetaState::ORIENTATION_Z);

  NormalRandomArrayGenerator<double> randomGenerator;
  randomGenerator.init(currentObservation.Y(),currentObservation.R());
  randomGenerator.fill(particleCourses);

  currentAddon.lastExteroceptiveUpdate.time=duration;
  currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;
}

}
