#ifndef _romea_ObservationRange_HPP_
#define _romea_ObservationRange_HPP_

#include <romea_filtering/GaussianObservation.hpp>

namespace romea {

  struct ObservationRange : GaussianObservation<double,1>
  {
    ObservationRange():
      responderPosition(Eigen::Vector3d::Zero()),
      initiatorPosition(Eigen::Vector3d::Zero()),
      terrainElevation(0)
    {

    }

    Eigen::Vector3d responderPosition;
    Eigen::Vector3d initiatorPosition;
    double terrainElevation;
  };
}

#endif
