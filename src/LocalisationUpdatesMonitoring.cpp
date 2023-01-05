// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <cmath>

// romea
#include "romea_core_localisation/LocalisationUpdatesMonitoring.hpp"


namespace romea
{


//---------------------------------------------------------------------------
LocalisationUpdateMonitoring::LocalisationUpdateMonitoring()
: time(Duration::zero()),
  travelledDistance(NAN)
{
}

}  // namespace romea
