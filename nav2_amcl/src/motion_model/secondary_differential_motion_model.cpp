#include "nav2_amcl/motion_model/secondary_differential_motion_model.hpp"
#include <iostream>

namespace nav2_amcl {

void
SecondaryDifferentialMotionModel::initialize(
  double alpha1, double alpha2, double alpha3, double alpha4, double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void
SecondaryDifferentialMotionModel::odometryUpdate(
  pf_t *pf, const pf_vector_t &pose, const pf_vector_t &delta)
{
  pf_sample_set_t *set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  //local_delta = rotation(old_pose) * delta

  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;

  pf_vector_t local_delta;
  double cos_theta = cos(-pose.v[2]);  
  double sin_theta = sin(-pose.v[2]);
  local_delta.v[0] = cos_theta * delta.v[0] - sin_theta * delta.v[1];  
  local_delta.v[1] = sin_theta * delta.v[0] + cos_theta * delta.v[1];

  if (sqrt(delta.v[1] * delta.v[1] + delta.v[0] * delta.v[0]) < 0.01) {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = angleutils::angle_diff(atan2(delta.v[1], delta.v[0]), old_pose.v[2]);
  }

  // Update delta_trans to use the new interpretation of delta.v[1] as forward movement
  delta_trans = sqrt(delta.v[1] * delta.v[1] + delta.v[0] * delta.v[0]);
  delta_rot2 = angleutils::angle_diff(delta.v[2], delta_rot1);

  // Calculate noise components
  delta_rot1_noise = std::min(fabs(angleutils::angle_diff(delta_rot1, 0.0)),
                              fabs(angleutils::angle_diff(delta_rot1, M_PI)));
  delta_rot2_noise = std::min(fabs(angleutils::angle_diff(delta_rot2, 0.0)),
                              fabs(angleutils::angle_diff(delta_rot2, M_PI)));

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t *sample = set->samples + i;

    delta_rot1_hat = angleutils::angle_diff(
      delta_rot1, pf_ran_gaussian(sqrt(alpha1_ * delta_rot1_noise * delta_rot1_noise +
                                       alpha2_ * delta_trans * delta_trans)));
    delta_trans_hat = delta_trans - pf_ran_gaussian(
      sqrt(alpha3_ * delta_trans * delta_trans + alpha4_ * delta_rot1_noise * delta_rot1_noise +
           alpha4_ * delta_rot2_noise * delta_rot2_noise));
    delta_rot2_hat = angleutils::angle_diff(
      delta_rot2, pf_ran_gaussian(sqrt(alpha1_ * delta_rot2_noise * delta_rot2_noise +
                                       alpha2_ * delta_trans * delta_trans)));

    // Adjust particle pose updates to match the new axis interpretation
    if ((std::abs(local_delta.v[0]) > std::abs(local_delta.v[1])) || (std::abs(local_delta.v[0]) < 1e-3 && std::abs(local_delta.v[1]) < 1e-3)) {  
      sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat); 
      sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat); 
    }
    else {
      sample->pose.v[0] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat); 
      sample->pose.v[1] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat); 
    }
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
  }
}

void
SecondaryDifferentialMotionModel::noiseOnlyUpdate(
  pf_t *pf, const pf_vector_t &pose, const pf_vector_t &delta)
{
  pf_sample_set_t *set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;

  pf_vector_t local_delta;
  double cos_theta = cos(-pose.v[2]);  
  double sin_theta = sin(-pose.v[2]);
  local_delta.v[0] = cos_theta * delta.v[0] - sin_theta * delta.v[1];  
  local_delta.v[1] = sin_theta * delta.v[0] + cos_theta * delta.v[1];

  if (sqrt(delta.v[1] * delta.v[1] + delta.v[0] * delta.v[0]) < 0.01) {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = angleutils::angle_diff(atan2(delta.v[1], delta.v[0]), old_pose.v[2]);
  }
  delta_trans = sqrt(delta.v[1] * delta.v[1] + delta.v[0] * delta.v[0]);
  delta_rot2 = angleutils::angle_diff(delta.v[2], delta_rot1);

  delta_rot1_noise = std::min(fabs(angleutils::angle_diff(delta_rot1, 0.0)),
                              fabs(angleutils::angle_diff(delta_rot1, M_PI)));
  delta_rot2_noise = std::min(fabs(angleutils::angle_diff(delta_rot2, 0.0)),
                              fabs(angleutils::angle_diff(delta_rot2, M_PI)));

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t *sample = set->samples + i;

    delta_rot1_hat = angleutils::normalize(
      pf_ran_gaussian(sqrt(alpha1_ * delta_rot1_noise * delta_rot1_noise +
                           alpha2_ * delta_trans * delta_trans)));
    delta_trans_hat = pf_ran_gaussian(sqrt(alpha3_ * delta_trans * delta_trans +
                                           alpha4_ * delta_rot1_noise * delta_rot1_noise +
                                           alpha4_ * delta_rot2_noise * delta_rot2_noise));
    delta_rot2_hat = angleutils::normalize(
      pf_ran_gaussian(sqrt(alpha1_ * delta_rot2_noise * delta_rot2_noise +
                           alpha2_ * delta_trans * delta_trans)));
    
    if ((std::abs(local_delta.v[0]) > std::abs(local_delta.v[1])) || (std::abs(local_delta.v[0]) < 1e-3 && std::abs(local_delta.v[1]) < 1e-3)){  
      sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat);
    }
    else{
      sample->pose.v[0] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat);
    }
    sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
  }
}

}  // namespace nav2_amcl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_amcl::SecondaryDifferentialMotionModel, nav2_amcl::MotionModel)
