// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFUPDATERPOSE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFUPDATERPOSE_HPP_


// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/ObservationPose.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea
{

class R2WLocalisationPFUpdaterPose
  : public LocalisationUpdaterExteroceptive,
  public PFGaussianUpdaterCore<double, 3, 3>
{
public:
  using Observation = ObservationPose;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;

public:
  R2WLocalisationPFUpdaterPose(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const size_t & numberOfParticles,
    const double & maximalMahalanobisDistance,
    const std::string & logFilename);

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    LocalisationFSMState & currentFSMState,
    MetaState & currentMetaState);

private:
  void computeInnovation_(
    const PFGaussianUpdaterCore::Observation & observation,
    const RawMajorVector & weights)override;

  void update_(
    const Duration & duration,
    Observation currentObservation,
    State & currentState,
    AddOn & currentAddon);


  bool set_(
    const Duration & duration,
    const Observation & currentObservation,
    const Input & currentInput,
    State & currentState,
    AddOn & currentAddon);

private:
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  LevelArmCompensation levelArmCompensation_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFUPDATERPOSE_HPP_
