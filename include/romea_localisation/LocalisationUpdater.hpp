#ifndef romea_LocalisationUpdaterFilter_hpp
#define romea_LocalisationUpdaterFilter_hpp

//std
#include <string>
#include <atomic>
#include <fstream>
#include <vector>

//romea
#include <romea_common/time/Time.hpp>
#include "LocalisationFSMState.hpp"

namespace romea {

class LocalisationUpdater
{

public :

  LocalisationUpdater(const std::string & logFilename,
                      const bool & disableUpdateFunction);

  virtual ~LocalisationUpdater()=default;

  void enableUpdateFunction();

  void disableUpdateFunction(const Duration & updateStartDisableTime = Duration::zero());

protected :

  void openLogFile_(const std::string & logFilename);

  void setLogFileHeader_(const std::vector<std::string> & logColumnNames);

protected :

  std::ofstream logFile_;
  Duration updateStartDisableTime_;
};

}//romea

#endif
