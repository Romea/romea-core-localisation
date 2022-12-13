#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"

namespace romea {

//-----------------------------------------------------------------------------
LocalisationUpdaterExteroceptive::LocalisationUpdaterExteroceptive(const std::string & updaterName,
                                                                   const double & minimalRate,
                                                                   const TriggerMode & triggerMode,
                                                                   const std::string & logFilename):
  LocalisationUpdaterBase(updaterName,
                          minimalRate,
                          triggerMode)
{
  openLogFile_(logFilename);
}


//-----------------------------------------------------------------------------
void LocalisationUpdaterExteroceptive::openLogFile_(const std::string & logFilename)
{
  if (!logFilename.empty())
  {
    logFile_.open(logFilename);

    if (!logFile_.is_open())
    {
      throw std::runtime_error("Cannot open debug file : "+logFilename);
    }
  }
}

//-----------------------------------------------------------------------------
void LocalisationUpdaterExteroceptive::setLogFileHeader_(
  const std::vector<std::string> & logColumnNames)
{
  if (logFile_.is_open())
  {
    logFile_ << "%";
    for (size_t n = 0; n < logColumnNames.size(); ++n)
    {
      logFile_ << "(" << n+1 << ")" << logColumnNames[n] << ",";
    }
    logFile_<< "\n";
  }
}


}  // namespace romea

