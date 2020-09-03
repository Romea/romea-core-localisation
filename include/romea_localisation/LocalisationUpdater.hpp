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

  LocalisationUpdater(const bool & disableUpdateFunction);

  virtual ~LocalisationUpdater()=default;

  void enableUpdateFunction();

  void disableUpdateFunction(const Duration & updateStartDisableTime = Duration::zero());

  void configureLogging(const std::string &logFilename, bool autostart=true);

  bool startLogging();

  void stopLogging();


protected :

  std::ofstream logFile_;
  std::vector<std::string> logColumnNames_;
  std::atomic<bool> isLogging_;

  Duration updateStartDisableTime_;
};

}//romea

#endif
