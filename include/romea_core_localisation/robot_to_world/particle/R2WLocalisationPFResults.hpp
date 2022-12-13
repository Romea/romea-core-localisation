#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFRESULTS_HPP_

// romea
#include <romea_core_filtering/particle/ParticleFilterEstimator.hpp>
#include "romea_core_localisation/robot_to_world/R2WLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea {

class R2WLocalisationPFResults : public R2WLocalisationResults<R2WLocalisationPFMetaState>
{
public :

  using  RowMajorMatrix = R2WLocalisationPFMetaState::State::RowMajorMatrix;

public:
  explicit R2WLocalisationPFResults(const size_t & numberOfParticles);
  virtual ~R2WLocalisationPFResults();

  const double & getX() const override;
  const double & getY() const override;
  const double & getYaw() const override;
  const double & getYawVariance() const override;

  Eigen::Vector3d getPose() const override;
  Eigen::Matrix3d getPoseCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  Pose2D toPose2D() const override;
  Pose3D toPose3D() const override;

  PoseAndTwist2D toPoseAndBodyTwist2D() const override;
  PoseAndTwist3D toPoseAndBodyTwist3D() const override;

  void reset(const Duration & duration);

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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFRESULTS_HPP_
