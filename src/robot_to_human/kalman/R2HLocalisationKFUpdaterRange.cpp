#include "romea_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterRange.hpp"

//Eigen
#include <Eigen/SVD>
#include <iostream>

//romea
#include <romea_common/math/Matrix.hpp>

namespace romea {

//-----------------------------------------------------------------------------
R2HLocalisationKFUpdaterRange::R2HLocalisationKFUpdaterRange(const std::string & logFileName,
                                                             const double &maximalMahalanobisDistance,
                                                             const bool usedConstraints):
  LocalisationUpdater(logFileName,false),
  KFUpdaterCore(maximalMahalanobisDistance),
  U_(Eigen::VectorXd::Zero(MetaState::STATE_SIZE)),
  W_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE,
                           MetaState::STATE_SIZE)),
  Amgs_(Eigen::Vector2d::Zero()),
  Tmgs_(Eigen::Matrix2d::Zero()),
  Wmgs_(0),
  Dc_(Eigen::Matrix2d::Zero(MetaState::STATE_SIZE,
                            MetaState::STATE_SIZE)),
  Yc_(Eigen::VectorXd::Zero(MetaState::STATE_SIZE)),
  RYc_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE,
                                 MetaState::STATE_SIZE)),

  isConstraintsUsed_(usedConstraints)
{
  Dc_(1,1)=1;

  setLogFileHeader_({"stamp",
                     "range",
                     "cov_range",
                     "x",
                     "y",
                     "cov_x",
                     "cov_xy",
                     "cov_y",
                     "ix",
                     "iy",
                     "apriori_range",
                     "cov_apriori_range",
                     "mahalanobis_distance",
                     "sucess"
                    });
}

//-----------------------------------------------------------------------------
void R2HLocalisationKFUpdaterRange::update(const Duration & duration,
                                           const Observation & currentObservation,
                                           LocalisationFSMState & currentFSMState,
                                           MetaState &currentMetaState)
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
void R2HLocalisationKFUpdaterRange::update_(const Duration &duration,
                                            const Observation & currentObservation,
                                            State & currentState,
                                            AddOn & currentAddOn)
{
  //compute observation matrix
  double aprioriRange = (currentState.X()-currentObservation.initiatorPosition.head<2>()).norm();
  H_ = (currentState.X()-currentObservation.initiatorPosition.head<2>()).transpose()/aprioriRange;
  double aprioriRangeVariance = (H_*currentState.P()*H_.transpose())(0,0);

  //Compute innovation
  Inn_ = currentObservation.Y()-aprioriRange;
  QInn_ = currentObservation.R() + aprioriRangeVariance;


  //Update state vector
  bool success = updateState_(currentState);

  if(logFile_.is_open())
  {
    logFile_<< duration.count()<<" ";
    logFile_<< currentObservation.Y() <<" ";
    logFile_<< currentObservation.R() << " ";
    logFile_<< currentState.X(0)<<",";
    logFile_<< currentState.X(1)<<",";
    logFile_<< currentState.P(0,0)<<",";
    logFile_<< currentState.P(0,1)<<",";
    logFile_<< currentState.P(1,1)<<",";
    logFile_<< currentObservation.initiatorPosition(0) <<",";
    logFile_<< currentObservation.initiatorPosition(1) <<",";
  }

  if(success){
    currentAddOn.lastExteroceptiveUpdate.time=duration;
    currentAddOn.lastExteroceptiveUpdate.travelledDistance=currentAddOn.travelledDistance;
  }

  //log
  if(logFile_.is_open())
  {
    logFile_<< aprioriRange <<",";
    logFile_<< aprioriRangeVariance <<",";
    logFile_<< this->mahalanobisDistance_ <<",";
    logFile_<< success <<",\n";
  }


  //  if(isConstraintsUsed_){

  //    //Covariance SVD decomposition
  //    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P, Eigen::ComputeThinU);

  //    U_=svd.matrixU();
  //    W_(0,0) =std::sqrt(svd.singularValues()(0));
  //    W_(1,1) =std::sqrt(svd.singularValues()(1));

  //    //Compute the modified Gram-Schmidt transformation Tmgs * Amgs = [ Wmgs ; 0 ].
  //    //A is a given n x m matrix, and S is an orthogonal n x n matrix, and W is an m x m matrix.
  //    Amgs_= W_*U_.transpose()*Dc_.row(0).transpose();
  //    Tmgs_ << Amgs_(0) , Amgs_(1) , -Amgs_(1), Amgs_(0);
  //    Tmgs_ /= Amgs_.norm();
  //    Wmgs_ = Amgs_.norm();

  //    //Lower and upper constraints
  //    double lowerConstraint = std::numeric_limits<double>::epsilon();
  //    double upperConstraint = 10000;

  //    // lower and upper mahalanobis distance
  //    double lowerMD = (lowerConstraint - X(0)) / std::sqrt(P(0,0));
  //    double upperMD = (upperConstraint - X(0)) / std::sqrt(P(0,0));

  //    double squaredLowerMD = lowerMD*lowerMD;
  //    double squaredUpperMD = upperMD*upperMD;

  //    //Truncated mean and variance of a posterior estimation
  //    double alpha = sqrt(2/M_PI) / (std::erf(upperMD/sqrt(2)) - std::erf(lowerMD/sqrt(2)));
  //    double mean = alpha * (std::exp(-squaredLowerMD/2) - std::exp(-squaredUpperMD/2));
  //    double var = alpha * (std::exp(-squaredLowerMD/2) * (lowerMD - 2 * mean));
  //    var-= alpha*(std::exp(-squaredUpperMD/2) * (upperMD - 2 *mean));
  //    var+= mean*mean+ 1;

  //    //  std::cout << " var " << var << std::endl;
  //    assert(var>0);

  //    //Constraint observation
  //    Yc_(0) = mean;
  //    RYc_(0,0) = var;

  //    //Update according constraints
  //    Tmgs_ = Tmgs_*std::sqrt(P(0,0))/Wmgs_;
  //    X = U_ * W_ * Tmgs_.transpose() * Yc_ + X;
  //    P = U_ * W_ * Tmgs_.transpose() * RYc_ * Tmgs_ * W_ * U_.transpose();

  //    assert(X(0)>0);
  //  }


  //std::cout <<  currentState.X() <<std::endl;
  //std::cout <<  currentState.P() <<std::endl;

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}

//-----------------------------------------------------------------------------
void R2HLocalisationKFUpdaterRange::useConstraints()
{
  isConstraintsUsed_=true;
}

}//romea
