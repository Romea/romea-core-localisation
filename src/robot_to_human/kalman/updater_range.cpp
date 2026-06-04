// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Eigen
#include <Eigen/SVD>

// romea
#include <romea_core_common/math/Matrix.hpp>

// std
#include <iostream>
#include <string>

// local
#include "romea_core_localisation/robot_to_human/kalman/updater_range.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2HKFUpdaterRange::R2HKFUpdaterRange(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const double & maximal_mahalanobis_distance,
  const std::string & logFilename,
  const bool & usedConstraints)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode, logFilename),
  EKFUpdaterBase<double, 2, 1>(maximal_mahalanobis_distance),
  U_(Eigen::VectorXd::Zero(MetaState::STATE_SIZE)),
  W_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  Amgs_(Eigen::Vector2d::Zero()),
  Tmgs_(Eigen::Matrix2d::Zero()),
  Wmgs_(0),
  Dc_(Eigen::Matrix2d::Zero(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  Yc_(Eigen::VectorXd::Zero(MetaState::STATE_SIZE)),
  RYc_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  isConstraintsUsed_(usedConstraints)
{
  Dc_(1, 1) = 1;

  set_log_file_header_(
    {"stamp",
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
     "sucess"});
}

//-----------------------------------------------------------------------------
void R2HKFUpdaterRange::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  rate_diagnostic_.evaluate(duration);

  if (current_fsm_State == FSMState::RUNNING) {
    try {
      update_(duration, current_observation, current_meta_state.state, current_meta_state.addon);
    } catch (...) {
      std::cout << " FSM : RANGE UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
      current_meta_state.state.reset();
      current_meta_state.addon.reset();
      current_fsm_State = FSMState::INIT;
    }
  }
}

//-----------------------------------------------------------------------------
void R2HKFUpdaterRange::update_(
  const Duration & duration,
  const Observation & current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // compute observation matrix
  double aprioriRange =
    (current_state.X() - current_observation.initiator_position.head<2>()).norm();
  H_ = (current_state.X() - current_observation.initiator_position.head<2>()).transpose() /
       aprioriRange;
  double aprioriRangeVariance = (H_ * current_state.P() * H_.transpose())(0, 0);

  // Compute innovation
  Inn_ = current_observation.Y() - aprioriRange;
  QInn_ = current_observation.R() + aprioriRangeVariance;

  // Update state vector
  bool success = update_state_(current_state);

  if (log_file_.is_open()) {
    log_file_ << duration.count() << " ";
    log_file_ << current_observation.Y() << " ";
    log_file_ << current_observation.R() << " ";
    log_file_ << current_state.X(0) << ",";
    log_file_ << current_state.X(1) << ",";
    log_file_ << current_state.P(0, 0) << ",";
    log_file_ << current_state.P(0, 1) << ",";
    log_file_ << current_state.P(1, 1) << ",";
    log_file_ << current_observation.initiator_position(0) << ",";
    log_file_ << current_observation.initiator_position(1) << ",";
  }

  if (success) {
    current_add_on.last_exteroceptive_update.time = duration;
    current_add_on.last_exteroceptive_update.travelled_distance = current_add_on.travelled_distance;
  }

  // log
  if (log_file_.is_open()) {
    log_file_ << aprioriRange << ",";
    log_file_ << aprioriRangeVariance << ",";
    log_file_ << this->mahalanobis_distance_ << ",";
    log_file_ << success << ",\n";
  }

  //  if(isConstraintsUsed_){

  //    //Covariance SVD decomposition
  //    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P, Eigen::ComputeThinU);

  //    U_=svd.matrixU();
  //    W_(0,0) =std::sqrt(svd.singularValues()(0));
  //    W_(1,1) =std::sqrt(svd.singularValues()(1));

  //    //Compute the modified Gram-Schmidt transformation Tmgs * Amgs = [ Wmgs
  //    ; 0 ].
  //    //A is a given n x m matrix, and S is an orthogonal n x n matrix, and W
  //    is an m x m matrix. Amgs_= W_*U_.transpose()*Dc_.row(0).transpose();
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
  //    double alpha = sqrt(2/M_PI) / (std::erf(upperMD/sqrt(2)) -
  //    std::erf(lowerMD/sqrt(2))); double mean = alpha *
  //    (std::exp(-squaredLowerMD/2) - std::exp(-squaredUpperMD/2)); double var
  //    = alpha * (std::exp(-squaredLowerMD/2) * (lowerMD - 2 * mean)); var-=
  //    alpha*(std::exp(-squaredUpperMD/2) * (upperMD - 2 *mean)); var+=
  //    mean*mean+ 1;

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

  // std::cout <<  current_state.X() <<std::endl;
  // std::cout <<  current_state.P() <<std::endl;

  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

//-----------------------------------------------------------------------------
void R2HKFUpdaterRange::useConstraints()
{
  isConstraintsUsed_ = true;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
