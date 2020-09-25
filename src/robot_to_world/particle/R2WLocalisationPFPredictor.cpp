//romea
#include "romea_localisation/robot_to_world/particle/R2WLocalisationPFPredictor.hpp"
#include <romea_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_common/math/EulerAngles.hpp>
#include <romea_common/math/Matrix.hpp>

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationPFPredictor::R2WLocalisationPFPredictor(const LocalisationStoppingCriteria & stoppingCriteria,
                                                       const size_t &numberOfParticles):
  LocalisationPredictor<MetaState>(stoppingCriteria),
  vxdT_(0),
  vydT_(0),
  cosCourses_(RowMajorVector::Zero(numberOfParticles)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles)),
  randomU_(RowMajorMatrix::Zero(3,numberOfParticles))
{
}

//--------------------------------------------------------------------------
void R2WLocalisationPFPredictor::predict_(const MetaState & previousMetaState,
                                          MetaState & currentMetaState)
{

  currentMetaState.input = previousMetaState.input;

  predictState_(previousMetaState.state,
                previousMetaState.input,
                currentMetaState.state);

  predictAddOn_(previousMetaState.addon,
                currentMetaState.addon);

//  assert(isPositiveSemiDefiniteMatrix(currentMetaState.state.P()));
  assert(isPositiveSemiDefiniteMatrix(currentMetaState.input.QU()));
}


//--------------------------------------------------------------------------
void R2WLocalisationPFPredictor::drawInputs(const Input &previousInput)
{
  vxdT_ = previousInput.U(MetaState::LINEAR_SPEED_X_BODY)*dt_;
  vydT_ = previousInput.U(MetaState::LINEAR_SPEED_Y_BODY)*dt_;

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(previousInput.U()*dt_,previousInput.QU()*dt_*dt_);
  randomGenerator.fill(randomU_);
}


//------------------------------------------------------------------------------
void R2WLocalisationPFPredictor::predictState_(const State &previousState,
                                               const Input &previousInput,
                                               State &currentState)
{
  drawInputs(previousInput);

  //predict particles
  currentState.particles.row(MetaState::ORIENTATION_Z) =
      previousState.particles.row(MetaState::ORIENTATION_Z)+
      randomU_.row(MetaState::ANGULAR_SPEED_Z_BODY);

  auto courses=currentState.particles.row(MetaState::ORIENTATION_Z);
  for(int n=0;n<previousState.particles.cols();++n)
      courses(n)=between0And2Pi(courses(n));


  cosCourses_=currentState.particles.row(MetaState::ORIENTATION_Z).cos();
  sinCourses_=currentState.particles.row(MetaState::ORIENTATION_Z).sin();

  currentState.particles.row(MetaState::POSITION_X) =
      previousState.particles.row(MetaState::POSITION_X)+
      cosCourses_*randomU_.row(MetaState::LINEAR_SPEED_X_BODY) -
      sinCourses_*randomU_.row(MetaState::LINEAR_SPEED_Y_BODY);

  currentState.particles.row(MetaState::POSITION_Y) =
      previousState.particles.row(MetaState::POSITION_Y)+
      sinCourses_*randomU_.row(MetaState::LINEAR_SPEED_X_BODY)+
      cosCourses_*randomU_.row(MetaState::LINEAR_SPEED_Y_BODY);


  currentState.weights = previousState.weights;

}

//------------------------------------------------------------------------------
void R2WLocalisationPFPredictor::predictAddOn_(const AddOn & previousAddOn,
                                               AddOn &currentAddOn)
{
  currentAddOn.roll=previousAddOn.roll;
  currentAddOn.pitch=previousAddOn.pitch;
  currentAddOn.roll=previousAddOn.rollPitchVariance;
  currentAddOn.travelledDistance =previousAddOn.travelledDistance + std::sqrt(vxdT_*vxdT_+vydT_*vydT_);
  currentAddOn.lastExteroceptiveUpdate = previousAddOn.lastExteroceptiveUpdate;
}

//-----------------------------------------------------------------------------
bool R2WLocalisationPFPredictor::stop_(const Duration & duration,
                                       const MetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration-metaState.addon.lastExteroceptiveUpdate.time;

  double travelledDistanceInDeadReckoningMode = metaState.addon.travelledDistance-metaState.addon.lastExteroceptiveUpdate.travelledDistance;


  //  double positionCircularErrorProbability = std::sqrt(state.P(R2WKalmanLocalisationState::POSITION_X,
  //                                                              R2WKalmanLocalisationState::POSITION_X)+
  //                                                      state.P(R2WKalmanLocalisationState::POSITION_Y,
  //                                                              R2WKalmanLocalisationState::POSITION_Y));

  double positionCircularErrorProbability = 0;


  return positionCircularErrorProbability > stoppingCriteria_.maximalPositionCircularErrorProbability ||
      travelledDistanceInDeadReckoningMode > stoppingCriteria_.maximalTravelledDistanceInDeadReckoning||
      durationInDeadReckoningMode > stoppingCriteria_.maximalDurationInDeadReckoning;

}



//-----------------------------------------------------------------------------
void R2WLocalisationPFPredictor::reset_(MetaState &metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}


}