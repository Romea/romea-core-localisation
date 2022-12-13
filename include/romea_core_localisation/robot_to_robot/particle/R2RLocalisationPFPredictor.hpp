#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFPREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFPREDICTOR_HPP_

// romea
#include "romea_core_localisation/LocalisationPredictor.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFMetaState.hpp"

namespace romea {

class R2RLocalisationPFPredictor : public  LocalisationPredictor<R2RLocalisationPFMetaState>
{
public:
  using MetaState = R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2RLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2RLocalisationPFMetaState::State::RowMajorMatrix;

public :

  R2RLocalisationPFPredictor(const Duration &maximalDurationInDeadReckoning,
                             const double &maximalTravelledDistanceInDeadReckoning,
                             const double &maximalPositionCircularErrorProbable,
                             const size_t & numberOfParticles);

private :

  bool stop_(const Duration & duration,
             const MetaState & meatastate)override;

  void predict_(const MetaState &previousMetaState,
                MetaState &currentMetaState)override;

  void reset_(MetaState &metaState)override;

  void predictState_(const State &previousState,
                     const Input &previousInput,
                     State &currentState);

  void predictAddOn_(const AddOn & previousAddOn,
                     const State &currentState,
                     AddOn &currentAddOn);


  void drawFollowerInputs(const Input & previousInput);

  void drawLeaderInputs(const Input & previousInput);

private:
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;

  double wfdT_, vxfdT_, vyfdT_;
  Eigen::Vector3d Uf_;
  Eigen::Matrix3d QUf_;
  Eigen::Vector3d Ufinv_;
  Eigen::Matrix3d QUfinv_;
  RowMajorMatrix randomUfinv_;

  Eigen::Vector3d Ul_;
  Eigen::Matrix3d QUl_;
  RowMajorMatrix randomUl_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_PARTICLE_R2RLOCALISATIONPFPREDICTOR_HPP_
