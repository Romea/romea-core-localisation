//romea
#include "romea_localisation/robot_to_robot/particle/R2RLocalisationPFResults.hpp"
#include <romea_common/math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
R2RLocalisationPFResults::R2RLocalisationPFResults(const size_t & numberOfParticles):
  R2RLocalisationResults<R2RLocalisationPFMetaState>(numberOfParticles),
  weightSum_(0),
  estimateStamp_(Duration::zero()),
  estimate_(Eigen::Vector3d::Zero()),
  estimateCovarianceStamp_(Duration::zero()),
  estimateCovariance_(Eigen::Matrix3d::Zero()),
  meanCenteredParticles_(R2RLocalisationPFMetaState::State::RowMajorMatrix::Zero(STATE_SIZE,numberOfParticles))
{

}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderX() const
{
  lazyComputeEstimate_();
  return estimate_(R2RLocalisationPFMetaState::LEADER_POSITION_X);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderY() const
{
  lazyComputeEstimate_();
  return estimate_(R2RLocalisationPFMetaState::LEADER_POSITION_Y);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderOrientation() const
{
  lazyComputeEstimate_();
  return estimate_(R2RLocalisationPFMetaState::LEADER_ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationPFResults::getLeaderPose() const
{
  lazyComputeEstimate_();
  return estimate_;

}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationPFResults::getLeaderPoseCovariance() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();
  return estimateCovariance_;
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLinearSpeed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLateralSpeed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getAngularSpeed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationPFResults::getTwist() const
{
  return input.U().segment<3>(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationPFResults::getTwistCovariance() const
{
  return input.QU().block<3,3>(LINEAR_SPEED_X_BODY,
                               LINEAR_SPEED_X_BODY);
}


//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderLinearSpeed() const
{
  return input.U(LEADER_LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderLateralSpeed() const
{
  return input.U(LEADER_LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RLocalisationPFResults::getLeaderAngularSpeed() const
{
  return input.U(LEADER_ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationPFResults::getLeaderTwist() const
{
  return input.U().segment<3>(LEADER_LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationPFResults::getLeaderTwistCovariance() const
{
  return input.QU().block<3,3>(LEADER_LINEAR_SPEED_X_BODY,
                               LEADER_LINEAR_SPEED_X_BODY);
}


//-----------------------------------------------------------------------------
void R2RLocalisationPFResults::lazyComputeEstimate_()const
{
  if(estimateStamp_!=duration_)
  {
    weightSum_ = state.weights.sum();
    this->estimate_(0) = (state.particles.row(0)*state.weights).sum()/weightSum_;
    this->estimate_(1) = (state.particles.row(1)*state.weights).sum()/weightSum_;
    double C=(state.particles.row(2).cos()*state.weights).sum()/weightSum_;
    double S=(state.particles.row(2).sin()*state.weights).sum()/weightSum_;
    this->estimate_(2)= std::atan2(S,C);

    estimateStamp_=duration_;
  }
}


//-----------------------------------------------------------------------------
void R2RLocalisationPFResults::lazyComputeEstimateVovariance_() const
{
  if(estimateCovarianceStamp_!=duration_)
  {
    for(int i=0;i<STATE_SIZE;++i)
      this->meanCenteredParticles_.row(i) = state.particles.row(i)- this->estimate_(i);

    //TODO à vectoriser
    auto courseRow = this->meanCenteredParticles_.row(2);
    for(int n=0;n<state.particles.cols();++n)
      courseRow(n) = betweenMinusPiAndPi(courseRow(n));


    for(int i=0 ; i<STATE_SIZE ; ++i)
      for(int j=i ; j<STATE_SIZE ; ++j)
        this->estimateCovariance_(i,j) =
          this->estimateCovariance_(j,i)= (this->meanCenteredParticles_.row(i)*
                                           this->meanCenteredParticles_.row(j)*
                                           state.weights).sum()/weightSum_;


    estimateCovarianceStamp_=duration_;
  }
}

//-----------------------------------------------------------------------------
Pose2D R2RLocalisationPFResults::toLeaderPose2D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  Pose2D pose2d;
  pose2d.position.x()=estimate_(LEADER_POSITION_X);
  pose2d.position.y()=estimate_(LEADER_POSITION_Y );
  pose2d.yaw =estimate_(LEADER_ORIENTATION_Z);
  pose2d.covariance = estimateCovariance_;
}


//-----------------------------------------------------------------------------
PoseAndTwist2D R2RLocalisationPFResults::toLeaderPoseAndBodyTwist2D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x()=estimate_(LEADER_POSITION_X);
  poseAndTwist2D.pose.position.y()=estimate_(LEADER_POSITION_Y);
  poseAndTwist2D.pose.yaw=estimate_(LEADER_ORIENTATION_Z);
  poseAndTwist2D.pose.covariance=estimateCovariance_;
  poseAndTwist2D.twist.linearSpeeds.x()=getLeaderLinearSpeed();
  poseAndTwist2D.twist.linearSpeeds.y()=getLeaderLateralSpeed();
  poseAndTwist2D.twist.angularSpeed=getLeaderAngularSpeed();
  poseAndTwist2D.twist.covariance=getLeaderTwistCovariance();
  return poseAndTwist2D;
}

}
