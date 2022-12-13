#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERPOSITION_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERPOSITION_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>
#include "romea_core_localisation/ObservationPosition.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"

namespace romea {


class R2WLocalisationPFUpdaterPosition :
  public LocalisationUpdaterExteroceptive,
  public PFGaussianUpdaterCore<double, 3, 2>
{
public :

  using Observation = ObservationPosition;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = Eigen::Array<double, 2, Eigen::Dynamic, Eigen::RowMajor>;

public :

  R2WLocalisationPFUpdaterPosition(const std::string & updaterName,
                                   const double & minimalRate,
                                   const TriggerMode & triggerMode,
                                   const size_t & numberOfParticles,
                                   const double &maximalMahalanobisDistance,
                                   const std::string & logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);

private :

  void update_(const Duration & duration,
               Observation currentObservation,
               State &currentState,
               AddOn &currentAddon);

  bool set_(const Duration & duration,
            const Observation & currentObservation,
            const Input & currentInput,
            State &currentState,
            AddOn &currentAddon);

  void computeLevelArms_(const Observation & ObservationPosition,
                         const AddOn & currentAddon);

  void setParticlePositions_(const Observation &ObservationPosition,
                             State &currentState);

  void applyLevelArmCompentations_(State & currentState);


private :

  RowMajorMatrix levelArms_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  LevelArmCompensation levelArmCompensation_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFUPDATERPOSITION_HPP_





