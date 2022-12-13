#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_R2HLOCALISATIONRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_R2HLOCALISATIONRESULTS_HPP_

// romea
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include <romea_core_common/geometry/Position2D.hpp>

namespace romea {

template <class MetaState>
class R2HLocalisationResults : public MetaState
{
public :

  template <typename ...Args>
  R2HLocalisationResults(Args ...args):
    MetaState(std::forward<Args>(args)...)
  {
  }

  virtual ~R2HLocalisationResults() = default;

  virtual const double & getLeaderX() const = 0;
  virtual const double & getLeaderY() const = 0;

  virtual Eigen::Vector2d getLeaderPosition() const = 0;
  virtual Eigen::Matrix2d getLeaderPositionCovariance() const = 0;

  virtual const double & getLinearSpeed() const = 0;
  virtual const double & getLateralSpeed() const = 0;
  virtual const double & getAngularSpeed() const = 0;

  virtual Eigen::Vector3d getTwist() const = 0;
  virtual Eigen::Matrix3d getTwistCovariance() const = 0;

  virtual Position2D toLeaderPosition2D() const = 0;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_HUMAN_R2HLOCALISATIONRESULTS_HPP_ 
