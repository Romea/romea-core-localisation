#ifndef _romea_R2HLocalisationKFUpdaterRange_HPP_
#define _romea_R2HLocalisationKFUpdaterRange_HPP_


//local
#include <romea_common/time/Time.hpp>
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include <romea_filtering/kalman/KalmanFilterUpdaterCore.hpp>
#include "R2HLocalisationKFMetaState.hpp"

namespace romea {

class R2HLocalisationKFUpdaterRange : public LocalisationUpdater, public KFUpdaterCore<double,2,1>
{
public :

  using Observation = ObservationRange;
  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public :

  R2HLocalisationKFUpdaterRange(const std::string & logFileName,
                                const double & maximalMahalanobisDistance,
                                const bool usedConstraints);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

  void useConstraints();

private :

  void update_(const Duration & duration,
               const Observation & currentObservation,
               State & currentState,
               AddOn & currentAddOn);


private :

  //Covariance Eigen Vector decomposition
  Eigen::MatrixXd U_;
  Eigen::MatrixXd W_;

  //Modified Grand Schmidt variable
  Eigen::MatrixXd Amgs_;
  Eigen::MatrixXd Tmgs_;
  double Wmgs_;

  //Constraint observation
  Eigen::MatrixXd Dc_;
  Eigen::VectorXd Yc_;
  Eigen::MatrixXd RYc_;

  bool isConstraintsUsed_;

};

}//romea
#endif
