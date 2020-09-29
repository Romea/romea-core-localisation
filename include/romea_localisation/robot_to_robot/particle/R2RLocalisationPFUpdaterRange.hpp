#ifndef romea_R2RLocalisationPFUpdaterRange_hpp
#define romea_R2RLocalisationPFUpdaterRange_hpp

//romea
#include <romea_common/time/Time.hpp>
#include "../../ObservationRange.hpp"
#include "../../LocalisationFSMState.hpp"
#include "../../LocalisationUpdater.hpp"
#include <romea_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>
#include <romea_filtering/particle/ParticleFilterResampling.hpp>
#include "R2RLocalisationPFMetaState.hpp"


namespace romea {


class R2RLocalisationPFUpdaterRange : public LocalisationUpdater, public PFGaussianUpdaterCore<double,3,1>
{
public:

  using Observation = ObservationRange;
  using MetaState = R2RLocalisationPFMetaState;
  using State = R2RLocalisationPFMetaState::State;
  using Input = R2RLocalisationPFMetaState::Input;
  using AddOn = R2RLocalisationPFMetaState::AddOn;
  using RowMajorVector = R2RLocalisationPFMetaState::State::RowMajorVector;

public :

  R2RLocalisationPFUpdaterRange(const size_t & numberOfParticles,
                                const double &maximalMahalanobisDistance,
                                const std::string &logFilename);

  void update(const Duration & duration,
              const Observation & currentObservation,
              LocalisationFSMState & currentFSMState,
              MetaState & currentMetaState);
protected :

  void update_(const Duration & duration,
               const Observation &currentObservation,
               State & currentState,
               AddOn & currentAddOn);

protected :

  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;

};

}

#endif
