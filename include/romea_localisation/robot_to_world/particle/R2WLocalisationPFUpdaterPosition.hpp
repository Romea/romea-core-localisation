#ifndef romea_R2WLocalisationPFUpdaterPosition_hpp
#define romea_R2WLocalisationPFUpdaterPosition_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>
#include "../../ObservationPosition.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include "../R2WLevelArmCompensation.hpp"
#include "R2WLocalisationPFMetaState.hpp"

namespace romea {


class R2WLocalisationPFUpdaterPosition : public LocalisationUpdater, public PFGaussianUpdaterCore<double,3,2>
{

public :

  using Observation = ObservationPosition;
  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = Eigen::Array<double,2,Eigen::Dynamic,Eigen::RowMajor>;

public :

  R2WLocalisationPFUpdaterPosition(const size_t & numberOfParticles,
                                   const double & maximalMahalanobisDistance,
                                   const bool &disableUpdateFunction,
                                   const std::string &logFilename);

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

  LevelArmCompensation levelArmCompensation_;
  RowMajorMatrix levelArms_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;


};

}

#endif





