#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONUPDATERLEADERTWIST_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONUPDATERLEADERTWIST_HPP_


// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <string>

// local
#include "../ObservationTwist.hpp"
#include "../LocalisationFSMState.hpp"
#include "../LocalisationUpdaterProprioceptive.hpp"

namespace romea
{
namespace core
{

template<class MetaState>
class R2RLocalisationUpdaterLeaderTwist : public LocalisationUpdaterProprioceptive
{
public:
  R2RLocalisationUpdaterLeaderTwist(
    const std::string & updaterName,
    const double & minimalRate)
  : LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  using Observation = ObservationTwist;

  virtual void update(
    const Duration & duration,
    const ObservationTwist & currentObservation,
    LocalisationFSMState & /*currentFSMState*/,
    MetaState & currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    currentMetaState.input.U().template
    segment<3>(MetaState::LEADER_LINEAR_SPEED_X_BODY) = currentObservation.Y();

    currentMetaState.input.QU().template
    block<3, 3>(
      MetaState::LEADER_LINEAR_SPEED_X_BODY,
      MetaState::LEADER_LINEAR_SPEED_X_BODY) = currentObservation.R();
  }
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__R2RLOCALISATIONUPDATERLEADERTWIST_HPP_
