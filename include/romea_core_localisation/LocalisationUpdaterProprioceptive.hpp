#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_

// std
#include <string>

// romea
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea {

class LocalisationUpdaterProprioceptive : public LocalisationUpdaterBase
{
public :

  LocalisationUpdaterProprioceptive(const std::string &updaterName,
                                    const double & minimalRate);

  virtual ~LocalisationUpdaterProprioceptive() = default;
};

} // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_
