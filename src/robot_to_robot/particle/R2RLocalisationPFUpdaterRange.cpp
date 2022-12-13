// romea
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFUpdaterRange.hpp"

namespace romea {

//--------------------------------------------------------------------------
R2RLocalisationPFUpdaterRange::R2RLocalisationPFUpdaterRange(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const size_t & numberOfParticles,
    const double &maximalMahalanobisDistance,
    const std::string & logFilename):
  LocalisationUpdaterExteroceptive(updaterName,
                                   minimalRate,
                                   triggerMode,
                                   logFilename),
  PFGaussianUpdaterCore(numberOfParticles,
                        maximalMahalanobisDistance),
  cosCourses_(RowMajorVector::Zero(numberOfParticles_)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles_))
{
}

//-----------------------------------------------------------------------------
void R2RLocalisationPFUpdaterRange::update(const Duration & duration,
                                           const Observation &currentObservation,
                                           LocalisationFSMState & currentFSMState,
                                           MetaState &currentMetaState)
{
  if (currentFSMState == LocalisationFSMState::RUNNING)
  {
    try
    {
      update_(duration,
              currentObservation,
              currentMetaState.state,
              currentMetaState.addon);
    }
    catch(...)
    {
      std::cout << " FSM : FILTER DEGENERESCENCE, RESET AND GO TO INIT MODE"<< std::endl;
      currentMetaState.state.reset();
      currentMetaState.addon.reset();
      currentFSMState = LocalisationFSMState::INIT;
    }
  }
}

//--------------------------------------------------------------------------
void R2RLocalisationPFUpdaterRange::update_(const Duration & duration,
                                            const Observation &currentObservation,
                                            State & currentState,
                                            AddOn & currentAddOn)
{
  // get position of the follower tag
  double xf = currentObservation.initiatorPosition.x();
  double yf = currentObservation.initiatorPosition.y();
  double zf = currentObservation.initiatorPosition.z();

  // get the position of leader tag
  double xl = currentObservation.responderPosition.x();
  double yl = currentObservation.responderPosition.y();
  double zl = currentObservation.responderPosition.z();

  // Compute importance weights
  const auto & courses = currentState.particles.row(2);
  const auto & x = currentState.particles.row(0);
  const auto & y = currentState.particles.row(1);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();

  aprioriObservations_ =((x+(cosCourses_*xl-sinCourses_*yl)-xf).square()+
                         (y+(sinCourses_*xl+cosCourses_*yl)-yf).square()+
                         (zl-zf)*(zl-zf)).sqrt();

  computeInnovation_(currentObservation, currentState.weights);
  bool success = updateState_(currentState, currentObservation);

  if (success)
  {
    currentAddOn.lastExteroceptiveUpdate.time = duration;
    currentAddOn.lastExteroceptiveUpdate.travelledDistance = currentAddOn.travelledDistance;
  }

  // log
  if (logFile_.is_open())
  {
    logFile_<< duration.count() << " ";
    logFile_<< success << " ";
    logFile_<< currentObservation.Y() << " ";
    logFile_<< currentObservation.R() << " ";
    logFile_<< aprioriObservation_.Y() << " ";
    logFile_<< aprioriObservation_.R() << " ";
    logFile_<< this->mahalanobisDistance_ <<std::endl;
  }
}

}  // namespace romea





