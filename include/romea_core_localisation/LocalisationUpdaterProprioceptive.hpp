// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_

// std
#include <string>

// romea
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea
{

class LocalisationUpdaterProprioceptive : public LocalisationUpdaterBase
{
public:
  LocalisationUpdaterProprioceptive(
    const std::string & updaterName,
    const double & minimalRate);

  virtual ~LocalisationUpdaterProprioceptive() = default;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERPROPRIOCEPTIVE_HPP_
