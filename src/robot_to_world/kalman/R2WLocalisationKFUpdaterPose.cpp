#include "romea_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterPose.hpp"


namespace romea {


//-----------------------------------------------------------------------------
R2WLocalisationKFUpdaterPose::R2WLocalisationKFUpdaterPose(const double & maximalMahalanobisDistance,
                                                           const bool &disableUpdateFunction):
  LocalisationUpdater(disableUpdateFunction),
  KFUpdaterCore(maximalMahalanobisDistance),
  levelArmCompensation_()
{
  H_(0, MetaState::POSITION_X)=1;
  H_(1, MetaState::POSITION_Y)=1;
  H_(2, MetaState::ORIENTATION_Z)=1;

  logColumnNames_ = {"stamp",
                     "x_obs",
                     "y_obs",
                     "theta_obs",
                     "cov_x_obs",
                     "cov_xy_obs",
                     "cov_xtheta_obs",
                     "cov_y_obs",
                     "cov_ytheta_obs",
                     "cov_theta_obs",
                     "x",
                     "y",
                     "theta",
                     "cov_x",
                     "cov_xy",
                     "cov_xtheta",
                     "cov_y",
                     "cov_ytheta",
                     "cov_theta",
                     "level_arm_x",
                     "level_arm_y"
                     "mahalanobis_distance",
                     "success"
                    };
}

//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterPose::update(const Duration &duration,
                                          const Observation &currentObservation,
                                          LocalisationFSMState & currentFSMState,
                                          MetaState &currentMetaState)
{
  switch (currentFSMState) {
  case LocalisationFSMState::INIT:
    if(set_(duration,
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
    if(duration<updateStartDisableTime_)
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
        std::cout << " FSM : POSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"<< std::endl;
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
void R2WLocalisationKFUpdaterPose::update_(const Duration & duration,
                                           const Observation & currentObservation,
                                           State &currentState,
                                           AddOn &currentAddon)
{
  //compute antenna attitude compensation
  levelArmCompensation_.compute(currentAddon.roll,
                                currentAddon.pitch,
                                currentAddon.rollPitchVariance,
                                currentState.X(MetaState::ORIENTATION_Z),
                                0, // orientation covariance is already in state covariance
                                currentObservation.levelArm);


  //Compute innovation
  this->Inn_[0]=currentObservation.Y(ObservationPose::POSITION_X)-currentState.X(MetaState::POSITION_X);
  this->Inn_[1]=currentObservation.Y(ObservationPose::POSITION_Y)-currentState.X(MetaState::POSITION_Y);
  this->Inn_[2]=betweenMinusPiAndPi(currentObservation.Y(ObservationPose::ORIENTATION_Z)-currentState.X(MetaState::ORIENTATION_Z));
  this->Inn_.template segment<2>(0) -= levelArmCompensation_.getPosition().segment<2>(0);

  //Compute innovation covariance
  //  this->R_ = currentObservation.R();
  //  this->R_.template block<2,2>(0,0) += levelArmCompensation_.getPositionCovariance().block<2,2>(0,0);
  this->H_.template block<2,1>(0,2)= levelArmCompensation_.getJacobian().block<2,1>(0,2);
  this->QInn_=this->H_*currentState.P()*this->H_.transpose()+currentObservation.R();
  this->QInn_.template block<2,2>(0,0)+=levelArmCompensation_.getPositionCovariance().block<2,2>(0,0);

  //log
  if(isLogging_)
  {
    logFile_<< duration.count()<<",";
    logFile_<< currentObservation.Y(0) <<",";
    logFile_<< currentObservation.Y(1) <<",";
    logFile_<< currentObservation.Y(2) <<",";
    logFile_<< currentObservation.R(0,0) << ",";
    logFile_<< currentObservation.R(0,1) << ",";
    logFile_<< currentObservation.R(0,2) << ",";
    logFile_<< currentObservation.R(1,1) << ",";
    logFile_<< currentObservation.R(1,2) << ",";
    logFile_<< currentObservation.R(2,2) << ",";
    logFile_<< currentState.X(0)<<",";
    logFile_<< currentState.X(1)<<",";
    logFile_<< currentState.X(2)<<",";
    logFile_<< currentState.P(0,0)<<",";
    logFile_<< currentState.P(0,1)<<",";
    logFile_<< currentState.P(0,2)<<",";
    logFile_<< currentState.P(1,1)<<",";
    logFile_<< currentState.P(1,2)<<",";
    logFile_<< currentState.P(2,2)<<",";
    logFile_<< levelArmCompensation_.getPosition()(0)<<",";
    logFile_<< levelArmCompensation_.getPosition()(1)<<",";
  }

  //Update state vector
  if(updateState_(currentState)){
    currentAddon.lastExteroceptiveUpdate.time=duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
  }

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}


//-----------------------------------------------------------------------------
bool R2WLocalisationKFUpdaterPose::set_(const Duration & duration,
                                        const ObservationPose & currentObservation,
                                        const Input & currentInput,
                                        State &currentState,
                                        AddOn &currentAddon)
{

  if(!std::isnan(currentInput.U(MetaState::LINEAR_SPEED_X_BODY))&&
     !std::isnan(currentInput.U(MetaState::LINEAR_SPEED_Y_BODY))&&
     !std::isnan(currentInput.U(MetaState::ANGULAR_SPEED_Z_BODY)))
  {
    currentState.X() = currentObservation.Y();
    currentState.P() = currentObservation.R();

    applyLevelArmCompensation(currentState,
                              currentAddon,
                              levelArmCompensation_,
                              currentObservation.levelArm);

    currentAddon.lastExteroceptiveUpdate.time=duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance = currentAddon.travelledDistance;
    return true;
  }
  else
  {
    return false;
  }
}

}





