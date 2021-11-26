#ifndef romea_LocalisationUpdaterProprioceptive_hpp
#define romea_LocalisationUpdaterProprioceptive_hpp

#include "LocalisationUpdaterBase.hpp"

namespace romea {

class LocalisationUpdaterProprioceptive : public LocalisationUpdaterBase
{

public :

  LocalisationUpdaterProprioceptive(const std::string &updaterName,
                                    const double & minimalRate);

  virtual ~LocalisationUpdaterProprioceptive()=default;

};

}//romea

#endif
