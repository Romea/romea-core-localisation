//romea
#include "romea_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterCourse.hpp"
#include <romea_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_common/math/EulerAngles.hpp>

//std
#include <random>

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationPFUpdaterCourse::R2WLocalisationPFUpdaterCourse(const size_t & numberOfParticles,
                                                               const double & /*maximalMahalanobisDistance*/,
                                                               const bool & disableUpdateFunction):
  LocalisationUpdater(disableUpdateFunction),
  PFUpdaterCore(numberOfParticles)
{

}


//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterCourse::update(const Duration & duration,
                                            const Observation & currentObservation,
                                            LocalisationFSMState & currentFSMState,
                                            MetaState & currentMetaState)
{
  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    set_(duration,
         currentObservation,
         currentMetaState.state,
         currentMetaState.addon);
    break;
  case LocalisationFSMState::RUNNING:
    //    update_(duration,currentObservation,currentState);
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
