#include "romea_localisation/LocalisationUpdater.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
LocalisationUpdater::LocalisationUpdater(const std::string & logFilename,
                                         const bool &disableUpdateFunction):
  logFile_(),
  updateStartDisableTime_(disableUpdateFunction ? Duration::zero():Duration::max())
{
  openLogFile_(logFilename);
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
void LocalisationUpdater::openLogFile_(const std::string &logFilename)
{
  if(logFilename.empty())
  {
    logFile_.open(logFilename);

    if(!logFile_.is_open())
    {
      throw std::runtime_error("Cannot open debug file : "+logFilename);
    }
  }
}

//-----------------------------------------------------------------------------
void LocalisationUpdater::setLogFileHeader_(const std::vector<std::string> & logColumnNames)
{
  if(logFile_.is_open())
  {
    logFile_<<"%";
    for(size_t n=0;n<logColumnNames.size();++n)
    {
      logFile_<<"("<<n+1<<")"<<logColumnNames[n]<<",";
    }
    logFile_<<"\n";
  }
}


////-----------------------------------------------------------------------------
//bool LocalisationUpdater::startLogging()
//{
//  if(logFile_.is_open())
//  {
//    isLogging_=true;
//  }

//  return isLogging_;
//}

////-----------------------------------------------------------------------------
//void LocalisationUpdater::stopLogging()
//{
//  isLogging_=false;
//}

}//romea
