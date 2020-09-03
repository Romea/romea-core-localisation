//romea
#include "romea_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterRange.hpp"
#include <romea_common/math/Matrix.hpp>

namespace {

const double UNSCENTED_TRANFORM_KAPPA =3;
const double UNSCENTED_TRANFORM_ALPHA =0.75;
const double UNSCENTED_TRANFORM_BETA = 2;

}

namespace romea {

//--------------------------------------------------------------------------
R2WLocalisationKFUpdaterRange::R2WLocalisationKFUpdaterRange(const double &maximalMahalanobisDistance,
                                                             const bool & disableUpdateFunction):
  LocalisationUpdater(disableUpdateFunction),
  UKFUpdaterCore(UNSCENTED_TRANFORM_KAPPA,
                 UNSCENTED_TRANFORM_ALPHA,
                 UNSCENTED_TRANFORM_BETA,
                 maximalMahalanobisDistance),
  antennaAtitudeCompensation_()
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
void R2WLocalisationKFUpdaterRange::update(const Duration & duration,
                                           const Observation &currentObservation,
                                           LocalisationFSMState & currentFSMState,
                                           MetaState & currentMetaState)
{

  //  std::cout << " update range " << std::endl;

  if(currentFSMState == LocalisationFSMState::RUNNING)
  {
    if(duration<updateStartDisableTime_)
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
        currentMetaState.state.reset();
        currentFSMState = LocalisationFSMState::INIT;
      }
    }
  }
}


//--------------------------------------------------------------------------
void R2WLocalisationKFUpdaterRange::update_(const Duration & duration,
                                            const Observation &currentObservation,
                                            State &currentState,
                                            AddOn &currentAddon)
{

  //compute antenna attitude compensation
  antennaAtitudeCompensation_.compute(currentAddon.roll,
                                      currentAddon.pitch,
                                      currentAddon.rollPitchVariance,
                                      0,
                                      0,
                                      currentObservation.initiatorPosition);


  const Eigen::Vector3d & tagAntennaPosition = antennaAtitudeCompensation_.getPosition();

  const double & ix =  tagAntennaPosition.x();
  const double & iy =  tagAntennaPosition.y();
  const double & iz =  tagAntennaPosition.z()+currentObservation.terrainElevation;

  const double & rx =  currentObservation.responderPosition.x();
  const double & ry =  currentObservation.responderPosition.y();
  const double & rz =  currentObservation.responderPosition.z();

  //compute sigma points
  computeStateSigmaPoints_(currentState);

  //progation of the sigma points
  for(size_t n=0; n<7;++n){
    const double & x = stateSigmaPoints_[n](0);
    const double & y = stateSigmaPoints_[n](1);
    const double coso = std::cos(stateSigmaPoints_[n](2));
        const double sino = std::sin(stateSigmaPoints_[n](2));

        propagatedSigmaPoints_[n]= std::sqrt(std::pow(x+ix*coso-iy*sino-rx,2)+
                                             std::pow(y+ix*sino+iy*coso-ry,2)+
                                             (iz-rz)*(iz-rz));

  }


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

  //update state
  bool success =updateState_(currentState,currentObservation);

  if(success)
  {
    currentAddon.lastExteroceptiveUpdate.time=duration;
    currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;
  }

  //log
  if(isLogging_)
  {
    logFile_<< this->propagatedState_.Y() <<" ";
    logFile_<< this->propagatedState_.R() <<" ";
    logFile_<< this->mahalanobisDistance_ <<"/n";
    logFile_<< success <<" ";
  }

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}

}





