#ifndef romea_R2RLocalisationPFPredictor_hpp
#define romea_R2RLocalisationPFPredictor_hpp

//romea
#include "../../LocalisationPredictor.hpp"
#include "R2RLocalisationPFMetaState.hpp"

namespace romea {

class R2RLocalisationPFPredictor : public  LocalisationPredictor<R2RLocalisationPFMetaState>
{

public:

  using MetaState =R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2RLocalisationPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2RLocalisationPFMetaState::State::RowMajorMatrix;

public :

  R2RLocalisationPFPredictor(const LocalisationStoppingCriteria & stoppingCriteria,
                             const size_t & numberOfParticles);

private :

  virtual bool stop_(const Duration & duration,
                     const MetaState & meatastate)override;

  virtual void predict_(const MetaState &previousMetaState,
                        MetaState &currentMetaState)override;

  virtual void reset_(MetaState &metaState)override;

  virtual void predictState_(const State &previousState,
                             const Input &previousInput,
                             State &currentState);

  virtual void predictAddOn_(const AddOn & previousAddOn,
                             const State &currentState,
                             AddOn &currentAddOn);


  virtual void drawFollowerInputs(const Input & previousInput);

  virtual void drawLeaderInputs(const Input & previousInput);

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

}



#endif
