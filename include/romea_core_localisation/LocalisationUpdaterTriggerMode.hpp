// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERTRIGGERMODE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERTRIGGERMODE_HPP_

#include <string>

namespace romea
{

enum class LocalisationUpdaterTriggerMode
{
  ALWAYS,
  ONCE
};

std::string toString(const LocalisationUpdaterTriggerMode & triggerMode);

LocalisationUpdaterTriggerMode toTriggerMode(const std::string & triggerMode);

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERTRIGGERMODE_HPP_
