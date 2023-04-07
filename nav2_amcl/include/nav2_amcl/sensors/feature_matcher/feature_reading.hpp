#ifndef NAV2_AMCL__SENSORS__FEATURE_READING_HPP_
#define NAV2_AMCL__SENSORS__FEATURE_READING_HPP_

#include <vector>

namespace nav2_amcl
{
  struct FeatureReading
  {
    double range;
    double bearing;
  };

  typedef std::vector<FeatureReading> FeatureReadings;
}

#endif  // NAV2_AMCL__SENSORS__FEATURE_READING_HPP_
