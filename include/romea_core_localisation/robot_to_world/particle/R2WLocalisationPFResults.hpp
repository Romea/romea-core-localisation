#ifndef romea_R2WLocalisationPFResults_hpp
#define romea_R2WLocalisationPFResults_hpp

//romea
#include <romea_core_filtering/particle/ParticleFilterEstimator.hpp>

#include "../R2WLocalisationResults.hpp"
#include "R2WLocalisationPFMetaState.hpp"

namespace romea {

class R2WLocalisationPFResults : public R2WLocalisationResults<R2WLocalisationPFMetaState>
{

public :

  using  RowMajorMatrix = R2WLocalisationPFMetaState::State::RowMajorMatrix;

public:

  R2WLocalisationPFResults(const size_t & numberOfParticles);
  virtual ~R2WLocalisationPFResults();

  virtual const double & getX() const override;
  virtual const double & getY() const override;
  virtual const double & getYaw() const override;
  virtual const double & getYawVariance() const override;

  virtual Eigen::Vector3d getPose() const override;
  virtual Eigen::Matrix3d getPoseCovariance() const override;

  virtual const double & getLinearSpeed() const override;
  virtual const double & getLateralSpeed() const override;
  virtual const double & getAngularSpeed() const override;

  virtual Eigen::Vector3d getTwist() const override;
  virtual Eigen::Matrix3d getTwistCovariance() const override;

  virtual Pose2D toPose2D() const override;
  virtual Pose3D toPose3D() const override;

  virtual PoseAndTwist2D toPoseAndBodyTwist2D() const override;
  virtual PoseAndTwist3D toPoseAndBodyTwist3D() const override;

  virtual void reset(const Duration & duration);

private :

  void lazyComputeEstimate_() const;
  void lazyComputeEstimateVovariance_() const;

private :

  mutable double weightSum_;

  mutable Duration estimateStamp_;
  mutable Eigen::Vector3d estimate_;

  mutable Duration estimateCovarianceStamp_;
  mutable Eigen::Matrix3d estimateCovariance_;

  mutable RowMajorMatrix meanCenteredParticles_;

};

}

#endif
