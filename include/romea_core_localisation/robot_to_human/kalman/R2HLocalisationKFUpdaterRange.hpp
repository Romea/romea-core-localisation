// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFUPDATERRANGE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFUPDATERRANGE_HPP_


// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp>

// std
#include <string>

//local
#include "romea_core_localisation/ObservationRange.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFMetaState.hpp"

namespace romea {

class R2HLocalisationKFUpdaterRange :
  public LocalisationUpdaterExteroceptive,
  public KFUpdaterCore<double, 2, 1>
{
public :

  using Observation = ObservationRange;
  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public :

  R2HLocalisationKFUpdaterRange(const std::string & updaterName,
                                const double & minimalRate,
                                const TriggerMode & triggerMode,
                                const double &maximalMahalanobisDistance,
                                const std::string & logFilename,
                                const bool & usedConstraints);

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

  // Covariance Eigen Vector decomposition
  Eigen::MatrixXd U_;
  Eigen::MatrixXd W_;

  // Modified Grand Schmidt variable
  Eigen::MatrixXd Amgs_;
  Eigen::MatrixXd Tmgs_;
  double Wmgs_;

  // Constraint observation
  Eigen::MatrixXd Dc_;
  Eigen::VectorXd Yc_;
  Eigen::MatrixXd RYc_;

  bool isConstraintsUsed_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFUPDATERRANGE_HPP_
