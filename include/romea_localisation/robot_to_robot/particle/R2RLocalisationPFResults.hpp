#ifndef _R2RLocalisationPFResults_hpp
#define _R2RLocalisationPFResults_hpp

#include "../R2RLocalisationResults.hpp"
#include "R2RLocalisationPFMetaState.hpp"

namespace romea {

class R2RLocalisationPFResults : public R2RLocalisationResults<R2RLocalisationPFMetaState>
{

public :

  R2RLocalisationPFResults(const size_t & numberOfParticles);
  virtual ~R2RLocalisationPFResults()=default;

  virtual const double & getLeaderX() const override;
  virtual const double & getLeaderY() const override;
  virtual const double & getLeaderOrientation() const override;

  virtual Eigen::Vector3d getLeaderPose() const override;
  virtual Eigen::Matrix3d getLeaderPoseCovariance() const override;

  virtual const double & getLinearSpeed() const override;
  virtual const double & getLateralSpeed() const override;
  virtual const double & getAngularSpeed() const override;

  virtual Eigen::Vector3d getTwist() const override;
  virtual Eigen::Matrix3d getTwistCovariance() const override;

  const double & getLeaderLinearSpeed() const override;
  const double & getLeaderLateralSpeed() const override;
  const double & getLeaderAngularSpeed() const override;

  Eigen::Vector3d getLeaderTwist() const override;
  Eigen::Matrix3d getLeaderTwistCovariance() const override;

  virtual Pose2D toLeaderPose2D() const override;
  virtual PoseAndTwist2D toLeaderPoseAndBodyTwist2D() const override;

private :

  void lazyComputeEstimate_() const;
  void lazyComputeEstimateVovariance_() const;

private :

  mutable double weightSum_;

  mutable Duration estimateStamp_;
  mutable Eigen::Vector3d estimate_;

  mutable Duration estimateCovarianceStamp_;
  mutable Eigen::Matrix3d estimateCovariance_;

  mutable R2RLocalisationPFMetaState::State::RowMajorMatrix meanCenteredParticles_;
};

}

#endif
