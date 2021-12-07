#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFPredictor.hpp"

//eigen
#include <Eigen/Geometry>

//romea
#include <romea_core_common/math/Matrix.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/containers/Eigen/EigenContainers.hpp>

namespace romea {

//-----------------------------------------------------------------------------
R2RLocalisationKFPredictor::R2RLocalisationKFPredictor(const LocalisationStoppingCriteria & stoppingCriteria):
  LocalisationPredictor<MetaState>(stoppingCriteria),
  jFl_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE,MetaState::STATE_SIZE)),
  jGl_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE,MetaState::INPUT_SIZE)),
  jFf_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE,MetaState::STATE_SIZE)),
  jGf_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE,MetaState::INPUT_SIZE)),
  xl_(0),
  yl_(0),
  thetal_(0),
  vxl_(0),
  vyl_(0),
  wl_(0),
  vxldT_(0),
  vyldT_(0),
  wldT_(0),
  dT_cos_thetal_wldT_(0),
  dT_sin_thetal_wldT_(0),
  vxf_(0),
  vyf_(0),
  wf_(0),
  vxfdT_(0),
  vyfdT_(0),
  wfdT_(0),
  dT_cos_wfdT_(0),
  dT_sin_wfdT_(0)
{
}

//-----------------------------------------------------------------------------
void R2RLocalisationKFPredictor::predict_(const R2RLocalisationKFMetaState & previousMetaState,
                                          R2RLocalisationKFMetaState &currentMetaState)
{

  currentMetaState.input = previousMetaState.input;

  predictState_(previousMetaState.state,
                previousMetaState.input,
                currentMetaState.state);

  predictAddOn_(previousMetaState.addon,
                currentMetaState.state,
                currentMetaState.addon);


  assert(isPositiveSemiDefiniteMatrix(currentMetaState.state.P()));
  assert(isPositiveSemiDefiniteMatrix(currentMetaState.input.QU()));
}


//-----------------------------------------------------------------------------
void R2RLocalisationKFPredictor::predictState_(const State &previousState,
                                               const Input &previousInput,
                                               State &currentState)
{

  //Predict state vector according leader displacement
  xl_     = previousState.X(MetaState::LEADER_POSITION_X);
  yl_     = previousState.X(MetaState::LEADER_POSITION_Y);
  thetal_ = previousState.X(MetaState::LEADER_ORIENTATION_Z);

  vxl_ = previousInput.U(MetaState::LEADER_LINEAR_SPEED_X_BODY);
  vxldT_ = vxl_*dt_;

  vyl_ = previousInput.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY);
  vyldT_ = vyl_*dt_;

  wl_ = previousInput.U(MetaState::LEADER_ANGULAR_SPEED_Z_BODY);
  wldT_ = wl_*dt_;

  dT_cos_thetal_wldT_ = dt_*std::cos(thetal_ + wldT_);
  dT_sin_thetal_wldT_ = dt_*std::sin(thetal_ + wldT_);

  currentState.X(MetaState::LEADER_POSITION_X) = xl_ +  vxl_*dT_cos_thetal_wldT_ - vyl_*dT_sin_thetal_wldT_;
  currentState.X(MetaState::LEADER_POSITION_Y)  = yl_ +  vxl_*dT_sin_thetal_wldT_ + vyl_*dT_cos_thetal_wldT_;
  currentState.X(MetaState::LEADER_ORIENTATION_Z) = betweenMinusPiAndPi(thetal_ + wldT_);

  //Predict state covariance
  jFl_(MetaState::LEADER_POSITION_X,
       MetaState::LEADER_POSITION_X)    = 1;
  jFl_(MetaState::LEADER_POSITION_X,
       MetaState::LEADER_ORIENTATION_Z)  = -vxl_*dT_sin_thetal_wldT_ - vyl_*dT_cos_thetal_wldT_;
  jFl_(MetaState::LEADER_POSITION_Y,
       MetaState::LEADER_POSITION_Y)     = 1;
  jFl_(MetaState::LEADER_POSITION_Y,
       MetaState::LEADER_ORIENTATION_Z)  =  vxl_*dT_cos_thetal_wldT_ - vyl_*dT_sin_thetal_wldT_;
  jFl_(MetaState::LEADER_ORIENTATION_Z,
       MetaState::LEADER_ORIENTATION_Z)  = 1;


  jGl_(MetaState::LEADER_POSITION_X,
       MetaState::LEADER_LINEAR_SPEED_X_BODY)  =  dT_cos_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y,
       MetaState::LEADER_LINEAR_SPEED_X_BODY)  = dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_X,
       MetaState::LEADER_LINEAR_SPEED_Y_BODY)  = -dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y,
       MetaState::LEADER_LINEAR_SPEED_Y_BODY)  =  dT_cos_thetal_wldT_;

  jGl_(MetaState::LEADER_POSITION_X,
       MetaState::LEADER_ANGULAR_SPEED_Z_BODY) =  -vxldT_*dT_sin_thetal_wldT_ - vyldT_*dT_cos_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y,
       MetaState::LEADER_ANGULAR_SPEED_Z_BODY) =   vxldT_*dT_cos_thetal_wldT_ - vyldT_*dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_ORIENTATION_Z,
       MetaState::LEADER_ANGULAR_SPEED_Z_BODY) =  dt_;

  currentState.P().noalias()  = jFl_*previousState.P()*jFl_.transpose();
  currentState.P().noalias() += jGl_*previousInput.QU()*jGl_.transpose();

  //Predict state vector according follower displacement
  vxf_ = previousInput.U(MetaState::LINEAR_SPEED_X_BODY);
  vxfdT_ = vxf_*dt_;

  vyf_ = previousInput.U(MetaState::LINEAR_SPEED_Y_BODY);
  vyfdT_ = vyf_*dt_;

  wf_ = previousInput.U(MetaState::ANGULAR_SPEED_Z_BODY);
  wfdT_ = wf_*dt_;

  dT_cos_wfdT_ = dt_*std::cos(-wfdT_);
  dT_sin_wfdT_ = dt_*std::sin(-wfdT_);

  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wfdT_));
  //  Eigen::Vector2d T = - R *Eigen::Vector2d(vxfdT_,vyfdT_);


  jFf_.block<2,2>(0,0) =R;
  jGf_.block<2,2>(0,0) = -R*dt_;
  jGf_(0,2) = vxfdT_*dT_sin_wfdT_ + vyfdT_*dT_cos_wfdT_;
  jGf_(1,2) = -vxfdT_*dT_cos_wfdT_ + vyfdT_*dT_sin_wfdT_;
  jGf_(2,2) = -dt_;

  currentState.X()= jFf_*(currentState.X() - Eigen::Vector3d(vxfdT_,vyfdT_,wfdT_));
  currentState.P()= jFf_*currentState.P()*jFf_.transpose() + jGf_*previousInput.QU()*jGf_.transpose();

}

//-----------------------------------------------------------------------------
void R2RLocalisationKFPredictor::predictAddOn_(const AddOn & previousAddOn,
                                               const State &currentState,
                                               AddOn &currentAddOn)
{

  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wfdT_));
  Eigen::Vector2d T = - R *Eigen::Vector2d(vxfdT_,vyfdT_);

  //Predict additional data
  transform(previousAddOn.robotTrajectory.get(),
            currentAddOn.robotTrajectory.get(),
            R,T);

  transform(previousAddOn.leaderTrajectory.get(),
            currentAddOn.leaderTrajectory.get(),
            R,T);

  currentAddOn.robotTrajectory.ringIndex_= previousAddOn.robotTrajectory.ringIndex_;
  currentAddOn.leaderTrajectory.ringIndex_= previousAddOn.leaderTrajectory.ringIndex_;

  if(currentAddOn.robotTrajectory.size()==0 ||
     currentAddOn.robotTrajectory[0].norm()>0.1)
  {
    currentAddOn.robotTrajectory.append(Eigen::Vector2d::Zero());
    currentAddOn.leaderTrajectory.append(currentState.X().head<2>());  }

  currentAddOn.travelledDistance = previousAddOn.travelledDistance+std::sqrt(vxfdT_*vxfdT_+vyfdT_*vyfdT_);
  currentAddOn.lastExteroceptiveUpdate = previousAddOn.lastExteroceptiveUpdate;
}

//-----------------------------------------------------------------------------
bool R2RLocalisationKFPredictor::stop_(const Duration & duration,
                                       const R2RLocalisationKFMetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration-metaState.addon.lastExteroceptiveUpdate.time;

  double travelledDistanceInDeadReckoningMode = metaState.addon.travelledDistance-metaState.addon.lastExteroceptiveUpdate.travelledDistance;

  double positionCircularErrorProbability = std::sqrt(metaState.state.P(MetaState::LEADER_POSITION_X,
                                                                        MetaState::LEADER_POSITION_X)+
                                                      metaState.state.P(MetaState::LEADER_POSITION_Y,
                                                                        MetaState::LEADER_POSITION_Y));

  //  std::cout << " kalman dr elapsed time "<< durationToSecond(duration) <<" "<< durationToSecond(state.lastExteroceptiveUpdate.time) <<" " <<  durationToSecond(stoppingCriteria_.maximalDurationInDeadReckoning)<< std::endl;
  //  std::cout << " kalman dr elapsed distance "<< state.travelledDistance <<" " <<state.lastExteroceptiveUpdate.travelledDistance <<" " <<  stoppingCriteria_.maximalTravelledDistanceInDeadReckoning<< std::endl;


  return positionCircularErrorProbability > stoppingCriteria_.maximalPositionCircularErrorProbability ||
      travelledDistanceInDeadReckoningMode > stoppingCriteria_.maximalTravelledDistanceInDeadReckoning||
      durationInDeadReckoningMode > stoppingCriteria_.maximalDurationInDeadReckoning;

}



//-----------------------------------------------------------------------------
void R2RLocalisationKFPredictor::reset_(R2RLocalisationKFMetaState &metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}//romea
