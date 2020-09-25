//romea
#include "romea_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterRange.hpp"

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationPFUpdaterRange::R2WLocalisationPFUpdaterRange(const size_t &numberOfParticles,
                                                             const double & maximalMahalanobisDistance,
                                                             const bool &disableUpdateFunction):
  LocalisationUpdater(disableUpdateFunction),
  PFGaussianUpdaterCore(numberOfParticles,maximalMahalanobisDistance),
  levelArmCompensation_(),
  cosCourses_(RowMajorVector::Zero(numberOfParticles_)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles_))
{
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterRange::update(const Duration & duration,
                                           const Observation & currentObservation,
                                           LocalisationFSMState & currentFSMState,
                                           MetaState &currentMetaState)
{

  if(currentFSMState == LocalisationFSMState::RUNNING)
  {
    if(duration<updateStartDisableTime_)
    {
      try{
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
}

//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterRange::update_(const Duration & duration,
                                            Observation currentObservation,
                                            State &currentState,
                                            AddOn &currentAddon)
{

  //compute antenna attitude compensation
  levelArmCompensation_.compute(currentAddon.roll,
                                currentAddon.pitch,
                                currentAddon.rollPitchVariance,
                                0,
                                0,
                                currentObservation.initiatorPosition);


  const Eigen::Vector3d & tagAntennaPosition = levelArmCompensation_.getPosition();

  //compute a priori observations
  const double & eax =  tagAntennaPosition.x();
  const double & eay =  tagAntennaPosition.y();
  const double & eaz =  tagAntennaPosition.z()+currentObservation.terrainElevation;

  const double & iax =  currentObservation.responderPosition.x();
  const double & iay =  currentObservation.responderPosition.y();
  const double & iaz =  currentObservation.responderPosition.z();

  const auto & courses = currentState.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = currentState.particles.row(MetaState::POSITION_X);
  const auto & y = currentState.particles.row(MetaState::POSITION_Y);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();

  aprioriObservations_ =((x+(cosCourses_*eax-sinCourses_*eay)-iax).square()+
                         (y+(sinCourses_*eax+cosCourses_*eay)-iay).square()+
                         (eaz-iaz)*(eaz-iaz)).sqrt();

  //update weights and resample
  currentObservation.R()+= levelArmCompensation_.getPositionCovariance().trace();

  if(updateState_(currentState,currentObservation))
  {
    currentAddon.lastExteroceptiveUpdate.time=duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
  }
}

}




