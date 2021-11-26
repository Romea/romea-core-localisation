#ifndef romea_LocalisationUpdaterExteroceptive_hpp
#define romea_LocalisationUpdaterExteroceptive_hpp

#include "romea_localisation/LocalisationUpdaterBase.hpp"

#include <fstream>

namespace romea {

class LocalisationUpdaterExteroceptive : public LocalisationUpdaterBase
{

public:

  LocalisationUpdaterExteroceptive(const std::string & updaterName,
                                   const double & minimalRate,
                                   const TriggerMode & triggerMode,
                                   const std::string & logFilename);

  virtual ~LocalisationUpdaterExteroceptive()=default;

  void openLogFile_(const std::string & logFilename);

  void setLogFileHeader_(const std::vector<std::string> & logColumnNames);

protected :

  std::ofstream logFile_;

};

}//romea

#endif
