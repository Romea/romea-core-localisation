#ifndef _R2HLocalisationKFResults_hpp
#define _R2HLocalisationKFResults_hpp

#include <romea_core_common/geometry/Position2D.hpp>
#include "../R2HLocalisationResults.hpp"
#include "R2HLocalisationKFMetaState.hpp"

namespace romea {

class R2HLocalisationKFResults : public R2HLocalisationResults<R2HLocalisationKFMetaState>
{

public :

  R2HLocalisationKFResults();

  virtual const double & getLeaderX() const;
  virtual const double & getLeaderY() const;

  virtual Eigen::Vector2d getLeaderPosition() const;
  virtual Eigen::Matrix2d getLeaderPositionCovariance() const;

  virtual const double & getLinearSpeed() const;
  virtual const double & getLateralSpeed() const;
  virtual const double & getAngularSpeed() const;

  virtual Eigen::Vector3d getTwist() const;
  virtual Eigen::Matrix3d getTwistCovariance() const;

  virtual Position2D toLeaderPosition2D() const override;

};

}

#endif
