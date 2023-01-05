// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_

// std
#include <fstream>
#include <vector>
#include <string>

// romea
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea
{

class LocalisationUpdaterExteroceptive : public LocalisationUpdaterBase
{
public:
  LocalisationUpdaterExteroceptive(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const std::string & logFilename);

  virtual ~LocalisationUpdaterExteroceptive() = default;

  void openLogFile_(const std::string & logFilename);

  void setLogFileHeader_(const std::vector<std::string> & logColumnNames);

protected:
  std::ofstream logFile_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_
