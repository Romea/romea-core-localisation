#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFPREDICTOR_HPP
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFPREDICTOR_HPP

// romea
#include "romea_core_localisation/LocalisationPredictor.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea {

class R2WLocalisationPFPredictor : public  LocalisationPredictor<R2WLocalisationPFMetaState>
{
public:

  using MetaState = R2WLocalisationPFMetaState;
  using State = R2WLocalisationPFMetaState::State;
  using Input = R2WLocalisationPFMetaState::Input;
  using AddOn = R2WLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2WLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2WLocalisationPFMetaState::State::RowMajorMatrix;

public :

  R2WLocalisationPFPredictor(const Duration & maximalDurationInDeadReckoning,
                             const double & maximalTravelledDistanceInDeadReckoning,
                             const double & maximalPositionCircularErrorProbable,
                             const size_t & numberOfParticles);

private :

  bool stop_(const Duration & duration,
             const MetaState & state)override;

  void predict_(const MetaState &previousMetaState,
                MetaState &currentMetaState)override;

  void reset_(MetaState &metaState)override;

  void predictState_(const State &previousState,
                     const Input &previousInput,
                     State &currentState);

  void predictAddOn_(const AddOn & previousAddOn,
                     AddOn &currentAddOn);

  void drawInputs(const Input &previousInput);

private:
  double vxdT_, vydT_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  RowMajorMatrix randomU_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_PARTICLE_R2WLOCALISATIONPFPREDICTOR_HPP
