#ifndef _romea_ObservationTwist_HPP_
#define _romea_ObservationTwist_HPP_

//romea
#include <romea_filtering/GaussianObservation.hpp>

namespace romea {

   struct ObservationTwist : GaussianObservation<double,3>
   {
     enum Index {
       LINEAR_SPEED_X_BODY = 0,
       LINEAR_SPEED_Y_BODY,
       ANGULAR_SPEED_Z_BODY,
       SIZE
     };

     Eigen::Vector3d levelArm;

   };

}


#endif
