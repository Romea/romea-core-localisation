#ifndef romea_R2WKalmanPredictor_hpp
#define romea_R2WKalmanPredictor_hpp

//romea
#include "../../LocalisationPredictor.hpp"
#include "R2WLocalisationKFMetaState.hpp"

namespace romea {

class R2WLocalisationKFPredictor : public  LocalisationPredictor<R2WLocalisationKFMetaState>
{

public :

  using MetaState = R2WLocalisationKFMetaState;
  using State = R2WLocalisationKFMetaState::State;
  using Input = R2WLocalisationKFMetaState::Input;
  using AddOn = R2WLocalisationKFMetaState::AddOn;

public :

  R2WLocalisationKFPredictor(const Duration &maximalDurationInDeadReckoning,
                             const double &maximalTravelledDistanceInDeadReckoning,
                             const double &maximalPositionCircularErrorProbable);

protected :

  virtual bool stop_(const Duration & duration,
                     const MetaState & state)override;

  virtual void predict_(const MetaState &previousState,
                        MetaState &nextState)override;

  virtual void reset_(MetaState &state)override;

  virtual void predictState_(const State &previousState,
                             const Input &previousInput,
                             State &currentState);

  virtual void predictAddOn_(const AddOn & previousAddOn,
                             AddOn &currentAddOn);

private:

  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;

  double x_, y_, theta_, vx_,vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_theta_wdT_;
  double dT_sin_theta_wdT_;

};

}



#endif
