// romea
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterPose.hpp"
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_common/math/EulerAngles.hpp>


namespace romea {

//-----------------------------------------------------------------------------
R2WLocalisationPFUpdaterPose::R2WLocalisationPFUpdaterPose(const std::string &updaterName,
                                                           const double &minimalRate,
                                                           const TriggerMode &triggerMode,
                                                           const size_t & numberOfParticles,
                                                           const double &maximalMahalanobisDistance,
                                                           const std::string &logFilename):
  LocalisationUpdaterExteroceptive(updaterName,
                                   minimalRate,
                                   triggerMode,
                                   logFilename),
  PFGaussianUpdaterCore(numberOfParticles,
                        maximalMahalanobisDistance),
  cosCourses_(RowMajorVector::Zero(numberOfParticles)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles)),
  levelArmCompensation_()
{
}


//--------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPose::update(const Duration &duration,
                                          const Observation &currentObservation,
                                          LocalisationFSMState & currentFSMState,
                                          MetaState &currentMetaState)
{
  rateDiagnostic_.evaluate(duration);

  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    if (set_(duration,
             currentObservation,
             currentMetaState.input,
             currentMetaState.state,
             currentMetaState.addon))
    {
      currentFSMState = LocalisationFSMState::RUNNING;
      std::cout << " FSM : INIT DONE (POSE), GO TO RUNNING MODE"<< std::endl;
    }
    break;
  case LocalisationFSMState::RUNNING:
    if (triggerMode_ == TriggerMode::ALWAYS)
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
        currentFSMState = LocalisationFSMState::INIT;
        currentMetaState.state.reset();
        currentMetaState.addon.reset();
      }
    }
    break;
  default:
    break;
  }
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPose::update_(const Duration & duration,
                                           Observation currentObservation,
                                           State & currentState,
                                           AddOn & currentAddon)
{
  // compute level arm compensation
  levelArmCompensation_.compute(currentAddon.roll,
                                currentAddon.pitch,
                                currentAddon.rollPitchVariance,
                                0,
                                0,
                                currentObservation.levelArm);

  double varxyantenna = levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0).trace();
  const Eigen::Vector3d & antennaPosition = levelArmCompensation_.getPosition();
  const double & xantenna =  antennaPosition(0);
  const double & yantenna =  antennaPosition(1);

  // compute apriori observations
  const auto & courses = currentState.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = currentState.particles.row(MetaState::POSITION_X);
  const auto & y = currentState.particles.row(MetaState::POSITION_Y);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();
  aprioriObservations_.row(MetaState::POSITION_X) = x+(cosCourses_*xantenna-sinCourses_*yantenna);
  aprioriObservations_.row(MetaState::POSITION_Y) = y+(sinCourses_*xantenna+cosCourses_*yantenna);
  aprioriObservations_.row(MetaState::ORIENTATION_Z) = courses;

  // update weights and resample
  currentObservation.R(MetaState::POSITION_X, MetaState::POSITION_X) += varxyantenna;
  currentObservation.R(MetaState::POSITION_Y, MetaState::POSITION_Y) += varxyantenna;
  if (updateState_(currentState, currentObservation))
  {
    currentAddon.lastExteroceptiveUpdate.time = duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
  }
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFUpdaterPose::computeInnovation_(
  const PFGaussianUpdaterCore::Observation &observation,
  const RawMajorVector & weights)
{
  double weightSum = weights.sum();

  aprioriObservation_.Y(MetaState::POSITION_X) =
    (aprioriObservations_.row(MetaState::POSITION_X)*weights).sum()/weightSum;
  aprioriObservation_.Y(MetaState::POSITION_Y) =
    (aprioriObservations_.row(MetaState::POSITION_Y)*weights).sum()/weightSum;
  aprioriObservation_.Y(MetaState::ORIENTATION_Z) =
    std::atan2((aprioriObservations_.row(MetaState::ORIENTATION_Z).sin()*weights).sum()/weightSum,
               (aprioriObservations_.row(MetaState::ORIENTATION_Z).cos()*weights).sum()/weightSum);

  for (int i = 0; i < 3 ; ++i)
  {
    aprioriMeanCenteredObservations_.row(i) = aprioriObservations_.row(i)-aprioriObservation_.Y(i);
  }

  // TODO(jean) à vectoriser
  for (size_t n = 0; n < numberOfParticles_; n++)
  {
    aprioriMeanCenteredObservations_(MetaState::ORIENTATION_Z, n)=
     betweenMinusPiAndPi(aprioriMeanCenteredObservations_(MetaState::ORIENTATION_Z, n));
  }

  for (size_t i = 0 ; i < 3 ; ++i){
    for (size_t j = i ; j < 3 ; ++j){
      aprioriObservation_.R(i, j) =
          aprioriObservation_.R(j, i)= (aprioriMeanCenteredObservations_.row(i)*
                                        aprioriMeanCenteredObservations_.row(j)*
                                        weights).sum()/weightSum;
    }
  }

  this->Inn_ = observation.Y() - aprioriObservation_.Y();
  this->QInn_ = observation.R() + aprioriObservation_.R();
}


//-----------------------------------------------------------------------------
bool R2WLocalisationPFUpdaterPose::set_(const Duration & duration,
                                        const Observation &currentObservation,
                                        const Input & currentInput,
                                        State &currentState,
                                        AddOn &currentAddon)
{
  if (!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY)) &&
      !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
      !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)))
  {
    Eigen::Vector3d pose = currentObservation.Y();
    Eigen::Matrix3d poseCovariance = currentObservation.R();

    levelArmCompensation_.compute(currentAddon.roll,
                                  currentAddon.pitch,
                                  currentAddon.rollPitchVariance,
                                  currentObservation.Y(ObservationPose::ORIENTATION_Z),
                                  currentObservation.R(ObservationPose::ORIENTATION_Z,
                                                       ObservationPose::ORIENTATION_Z),
                                  currentObservation.levelArm);

    pose.segment<2>(0) -= levelArmCompensation_.getPosition().segment<2>(0);
    poseCovariance.block<2, 2>(0, 0) += levelArmCompensation_.getPositionCovariance().block<2, 2>(0, 0);

    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(pose, poseCovariance);
    randomGenerator.fill(currentState.particles);

    currentAddon.lastExteroceptiveUpdate.time = duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;

    return true;
  } else {
    return false;
  }
}

}  // namespace romea





