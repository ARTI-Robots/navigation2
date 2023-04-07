#ifndef NAV2_AMCL__SENSORS__FEATURE_MODEL_HPP_
#define NAV2_AMCL__SENSORS__FEATURE_MODEL_HPP_

#include <nav2_amcl/sensors/feature_matcher/feature_reading.hpp>
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include <nav2_amcl/kdtree/kd_tree_point.h>
#include <memory>

namespace nav2_amcl
{
/*
 * @class FeatureModel
 * @brief Simple likelihood field model for feature matching
 */
class FeatureModel
{
public:
  /**
   * @brief A FeatureModel constructor
   * @param feature_map map of feature positions used for matching
   * @param sigma_hit sigma used for scaling a seen feature
   */
  FeatureModel(const Points & feature_map, double sigma_hit);

  /*
   * @brief FeatureModel destructor
   */
  virtual ~FeatureModel() = default;

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, FeatureReadings* data);

  /*
   * @brief Run a sensor update on laser for a sample set
   * @param pf Particle filter to use
   * @param data feature data to use
   * @return if it was succesful
   */
  double sensorUpdateOnSampleSet(pf_sample_set_t * pf, FeatureReadings* data);
  
  struct CallHelper
  {
       FeatureModel* object;
       FeatureReadings* real_data;
  };

  static double sensorFunction(void * data, pf_sample_set_t * set);

protected:
  double sqare_sigma_;
  double scaling_;
  PointSet point_set_;
  KDTreePoint kd_tree_;
};
}

#endif  // NAV2_AMCL__SENSORS__FEATURE_MODEL_HPP_
