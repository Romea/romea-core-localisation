#include "romea_localisation/robot_to_robot/kalman/R2RLocalisationKFUpdaterRange.hpp"

//Eigen
#include <Eigen/SVD>
#include <Eigen/LU>

//romea
#include <romea_common/math/Matrix.hpp>

//std
#include <iostream>

namespace {
const double UNSCENTED_TRANSFORM_KAPPA=3;
const double UNSCENTED_TRANSFORM_ALPHA=0.75;
const double UNSCENTED_TRANSFORM_BETA=2;
}

namespace romea {

//-----------------------------------------------------------------------------
R2RLocalisationKFUpdaterRange::R2RLocalisationKFUpdaterRange(const double &maximalMahalanobisDistance):
  LocalisationUpdater(false),
  UKFUpdaterCore(UNSCENTED_TRANSFORM_KAPPA,
                 UNSCENTED_TRANSFORM_ALPHA,
                 UNSCENTED_TRANSFORM_BETA,
                 maximalMahalanobisDistance)
{
  logColumnNames_ = {"stamp",
                     "range",
                     "cov_range",
                     "x",
                     "y",
                     "theta",
                     "cov_x",
                     "cov_xy",
                     "cov_xtheta",
                     "cov_y",
                     "cov_ytheta",
                     "cov_theta",
                     "ix",
                     "iy",
                     "iz",
                     "rx",
                     "ry",
                     "rz",
                     "apriori_range",
                     "cov_apriori_range",
                     "mahalanobis_distance",
                     "sucess"
                    };

}

//-----------------------------------------------------------------------------
void R2RLocalisationKFUpdaterRange::update(const Duration & duration,
                                           const Observation & currentObservation,
                                           LocalisationFSMState & currentFSMState,
                                           MetaState & currentMetaState)
{
  if(currentFSMState == LocalisationFSMState::RUNNING)
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
      std::cout << " FSM : RANGE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"<< std::endl;
      currentMetaState.state.reset();
      currentMetaState.addon.reset();
      currentFSMState = LocalisationFSMState::INIT;
    }
  }
}


//-----------------------------------------------------------------------------
void R2RLocalisationKFUpdaterRange::update_(const Duration &duration,
                                            const Observation & currentObservation,
                                            State & currentState,
                                            AddOn & currentAddOn)
{

  //compute sigma points
  computeStateSigmaPoints_(currentState);

  //get position of the follower tag
  double ix = currentObservation.initiatorPosition.x();
  double iy = currentObservation.initiatorPosition.y();
  double iz = currentObservation.initiatorPosition.z();

  //get the position of leader tag
  double rx = currentObservation.responderPosition.x();
  double ry = currentObservation.responderPosition.y();
  double rz = currentObservation.responderPosition.z();

  //propagation of the sigma points
  for(size_t n=0; n<7;++n)
  {
    const double & x = stateSigmaPoints_[n](0);
    const double & y = stateSigmaPoints_[n](1);
    const double coso = std::cos(stateSigmaPoints_[n](2));
        const double sino = std::sin(stateSigmaPoints_[n](2));

        propagatedSigmaPoints_[n] = std::sqrt(std::pow(x+rx*coso-ry*sino-ix,2)+
                                              std::pow(y+rx*sino+ry*coso-iy,2)+
                                              (rz-iz)*(rz-iz));

  }

  //update state
  bool success = updateState_(currentState,currentObservation);

  if(isLogging_)
  {
    logFile_<< duration.count()<<" ";
    logFile_<< currentObservation.Y() <<" ";
    logFile_<< currentObservation.R() << " ";
    logFile_<< currentState.X(0)<<",";
    logFile_<< currentState.X(1)<<",";
    logFile_<< currentState.X(2)<<",";
    logFile_<< currentState.P(0,0)<<",";
    logFile_<< currentState.P(0,1)<<",";
    logFile_<< currentState.P(0,1)<<",";
    logFile_<< currentState.P(1,1)<<",";
    logFile_<< currentState.P(1,2)<<",";
    logFile_<< currentState.P(2,2)<<",";
    logFile_<< ix <<",";
    logFile_<< iy <<",";
    logFile_<< iz <<",";
    logFile_<< rx <<",";
    logFile_<< ry <<",";
    logFile_<< rz <<",";
  }

  if(success)
  {
    currentAddOn.lastExteroceptiveUpdate.time=duration;
    currentAddOn.lastExteroceptiveUpdate.travelledDistance=currentAddOn.travelledDistance;
  }

  //log
  if(isLogging_)
  {
    logFile_<< this->propagatedState_.Y() <<" ";
    logFile_<< this->propagatedState_.R() <<" ";
    logFile_<< this->mahalanobisDistance_ <<std::endl;
    logFile_<< success <<" ";
  }

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));

}

}//romea
