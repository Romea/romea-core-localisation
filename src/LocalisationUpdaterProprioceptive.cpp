#include "romea_core_localisation/LocalisationUpdaterProprioceptive.hpp"

namespace romea {

//-----------------------------------------------------------------------------
LocalisationUpdaterProprioceptive::LocalisationUpdaterProprioceptive(const std::string &updaterName,
                                                                     const double & minimalRate):
  LocalisationUpdaterBase(updaterName,
                          minimalRate,
                          TriggerMode::ALWAYS)
{
}

} // namespace romea

