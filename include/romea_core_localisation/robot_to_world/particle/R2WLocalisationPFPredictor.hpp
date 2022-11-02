#ifndef romea_R2WLocalisationPFPredictor_hpp
#define romea_R2WLocalisationPFPredictor_hpp

//romea
#include "../../LocalisationPredictor.hpp"
#include "R2WLocalisationPFMetaState.hpp"

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

  virtual bool stop_(const Duration & duration,
                     const MetaState & state)override;

  virtual void predict_(const MetaState &previousMetaState,
                        MetaState &currentMetaState)override;

  virtual void reset_(MetaState &metaState)override;

  virtual void predictState_(const State &previousState,
                             const Input &previousInput,
                             State &currentState);

  virtual void predictAddOn_(const AddOn & previousAddOn,
                             AddOn &currentAddOn);

  virtual void drawInputs(const Input &previousInput);

private:

  double vxdT_, vydT_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  RowMajorMatrix randomU_;

};

}



#endif
