#include "romea_localisation/LocalisationUpdater.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
LocalisationUpdater::LocalisationUpdater(const bool &disableUpdateFunction):
  logFile_(),
  logColumnNames_(),
  isLogging_(false),
  updateStartDisableTime_(disableUpdateFunction ? Duration::zero():Duration::max())
{

}

//-----------------------------------------------------------------------------
void LocalisationUpdater::enableUpdateFunction()
{
  updateStartDisableTime_=Duration::max();
}

//-----------------------------------------------------------------------------
void LocalisationUpdater::disableUpdateFunction(const Duration & updateStartDisableTime)
{
  updateStartDisableTime_=updateStartDisableTime;
}

//-----------------------------------------------------------------------------
void LocalisationUpdater::configureLogging(const std::string &logFilename,bool autostart)
{
  logFile_.open(logFilename);

  if(!logFile_.is_open())
  {
    throw std::runtime_error("Cannot open debug file : "+logFilename);
  }

  if(!logColumnNames_.empty())
  {
    logFile_<<"%";
    for(size_t n=0;n<logColumnNames_.size();++n)
    {
      logFile_<<"("<<n+1<<")"<<logColumnNames_[n]<<",";
    }
    logFile_<<"\n";
  }
  isLogging_=autostart;
}

//-----------------------------------------------------------------------------
bool LocalisationUpdater::startLogging()
{
  if(logFile_.is_open())
  {
    isLogging_=true;
  }

  return isLogging_;
}

//-----------------------------------------------------------------------------
void LocalisationUpdater::stopLogging()
{
  isLogging_=false;
}

}//romea
