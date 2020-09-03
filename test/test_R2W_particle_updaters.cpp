// gtest
#include <gtest/gtest.h>

#include "filter/LocalisationUpdaterAngularSpeed.hpp"
#include "filter/LocalisationUpdaterLinearSpeed.hpp"
#include "filter/LocalisationUpdaterLinearSpeeds.hpp"
#include "filter/LocalisationUpdaterTwist.hpp"

class TestProprioceptiveUpdate : public ::testing::Test
{
public:

  TestProprioceptiveUpdate()

};

TEST_(TestLinearSpeedConverionObservationConversion, fromRosMsgToObs)
{

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
