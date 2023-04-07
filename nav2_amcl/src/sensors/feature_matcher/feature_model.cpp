#include <nav2_amcl/sensors/feature_matcher/feature_model.hpp>
#include <functional>

namespace nav2_amcl
{
  FeatureModel::FeatureModel(const Points & feature_map, double sigma_hit) : point_set_(feature_map),
  kd_tree_(2, point_set_, nanoflann::KDTreeSingleIndexAdaptorParams())
  {
    kd_tree_.buildIndex();
    sqare_sigma_ = std::pow(sigma_hit, 2.);
    scaling_ = 1./std::sqrt(2.*M_PI*sqare_sigma_);    
  }

  bool FeatureModel::sensorUpdate(pf_t * pf, FeatureReadings* data)
  {
    CallHelper used_data;
    used_data.object = this;
    used_data.real_data = data;
    pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, &used_data);
    return true;    
  }

  double FeatureModel::sensorUpdateOnSampleSet(pf_sample_set_t * set, FeatureReadings* data)
  {
    double total_weight = 0.0;

    
    // Pre-compute a couple of things
    for (int j = 0; j < set->sample_count; j++)
    {
        pf_sample_t * sample = set->samples + j;
        pf_vector_t pose = sample->pose;

        sample->weight = 0.;

      for (const auto &reading : *data)
      {
        // Compute the endpoint of the beam
        double query_point[2];
        query_point[0] = pose.v[0] + reading.range * cos(pose.v[2] + reading.bearing);
        query_point[1] = pose.v[1] + reading.range * sin(pose.v[2] + reading.bearing);        

        std::vector<size_t> ret_index(1);
        std::vector<double> out_dist_sqr(1);
        size_t num_results = kd_tree_.knnSearch(&query_point[0], 1, &ret_index[0], &out_dist_sqr[0]);
        if (num_results == 1)
        {
          sample->weight += std::exp(-0.5*out_dist_sqr[0] / sqare_sigma_);
        } 
      }

      sample->weight /= scaling_;

      total_weight += sample->weight;
    }

    return total_weight;
  }

  double FeatureModel::sensorFunction(void * data, pf_sample_set_t * set)
  {
    CallHelper* casted_data = reinterpret_cast<CallHelper*>(data);
    return casted_data->object->sensorUpdateOnSampleSet(set, casted_data->real_data);
  }
}