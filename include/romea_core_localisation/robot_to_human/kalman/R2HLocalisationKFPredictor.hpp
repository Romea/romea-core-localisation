#ifndef _romea_R2HLocalisationKFPredictor_HPP_
#define _romea_R2HLocalisationKFPredictor_HPP_

#include "R2HLocalisationKFMetaState.hpp"
#include "../../LocalisationPredictor.hpp"

namespace romea {

class R2HLocalisationKFPredictor : public LocalisationPredictor<R2HLocalisationKFMetaState>
{
public :

  using MetaState = R2HLocalisationKFMetaState;
  using State = R2HLocalisationKFMetaState::State;
  using Input = R2HLocalisationKFMetaState::Input;
  using AddOn = R2HLocalisationKFMetaState::AddOn;

public :

  R2HLocalisationKFPredictor(const LocalisationStoppingCriteria &stoppingCriteria,
                             const Eigen::Matrix2d & leaderMotionCovariance);

  virtual ~R2HLocalisationKFPredictor()=default;

private :

  virtual bool stop_(const Duration  & duration,
                     const MetaState &state)override;

  virtual void predict_(const MetaState &previousMetaState,
                        MetaState &currentMetaState)override;

  virtual void reset_(MetaState & metaState)override;


private :

  void predictState_(const State &previousState,
                     const Input &prviousInput,
                     State &currentState);

  void predictAddOn_(const AddOn & previousAddOn,
                     const State &currentState,
                     AddOn &currentAddOn);

private :

  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;
  Eigen::MatrixXd leaderMotionCovariance_;

  double vx_, vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_wdT_;
  double dT_sin_wdT_;

};

}//romea

#endif
