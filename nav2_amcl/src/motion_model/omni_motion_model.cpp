/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include <iostream>

#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/angleutils.hpp"

namespace nav2_amcl
{

OmniMotionModel::OmniMotionModel(
  double alpha1, double alpha2, double alpha3, double alpha4,
  double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void
OmniMotionModel::odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta)
{
  // Compute the new sample poses
  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  double delta_trans, delta_rot, delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

  delta_trans = sqrt(
    delta.v[0] * delta.v[0] +
    delta.v[1] * delta.v[1]);
  delta_rot = delta.v[2];

  // Precompute a couple of things
  double trans_hat_stddev = sqrt(
    alpha3_ * (delta_trans * delta_trans) +
    alpha4_ * (delta_rot * delta_rot) );
  double rot_hat_stddev = sqrt(
    alpha1_ * (delta_rot * delta_rot) +
    alpha2_ * (delta_trans * delta_trans) );
  double strafe_hat_stddev = sqrt(
    alpha4_ * (delta_rot * delta_rot) +
    alpha5_ * (delta_trans * delta_trans) );

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    delta_bearing = angleutils::angle_diff(
      atan2(delta.v[1], delta.v[0]),
      old_pose.v[2]) + sample->pose.v[2];
    double cs_bearing = cos(delta_bearing);
    double sn_bearing = sin(delta_bearing);

    // Sample pose differences
    delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
    delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
    delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
    // Apply sampled update to particle pose
    sample->pose.v[0] += (delta_trans_hat * cs_bearing +
      delta_strafe_hat * sn_bearing);
    sample->pose.v[1] += (delta_trans_hat * sn_bearing -
      delta_strafe_hat * cs_bearing);
    sample->pose.v[2] += delta_rot_hat;
  }
}

void
OmniMotionModel::noiseOnlyUpdate(pf_t * pf __attribute__((unused)), const pf_vector_t & pose __attribute__((unused)), const pf_vector_t & delta __attribute__((unused)))
{

  std::cout << "NoiseOnlyUpdate called with delta " << delta.v[0] << " " << delta.v[1] << " " << delta.v[2]
            << std::endl;
  std::cout << "alpha1 " << alpha1_ << " alpha2 " << alpha2_ << " alpha3 " << alpha3_ << " alpha4 " << alpha4_
            << " alpha5 " << alpha5_ << std::endl;
  std::cout << "pose " << pose.v[0] << " " << pose.v[1] << " " << pose.v[2] << std::endl;

  // Compute the new sample poses
  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  double delta_trans, delta_rot, delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

  delta_trans = sqrt(
   delta.v[0] * delta.v[0] +
   delta.v[1] * delta.v[1]);
  delta_rot = delta.v[2];


  // Precompute a couple of things
  double trans_hat_stddev = sqrt(
   alpha3_ * (delta_trans * delta_trans) +
   alpha4_ * (delta_rot * delta_rot) );
  double rot_hat_stddev = sqrt(
   alpha1_ * (delta_rot * delta_rot) +
   alpha2_ * (delta_trans * delta_trans) );
  double strafe_hat_stddev = sqrt(
   alpha4_ * (delta_rot * delta_rot) +
   alpha5_ * (delta_trans * delta_trans) );

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    delta_bearing = angleutils::angle_diff(
     atan2(delta.v[1], delta.v[0]),
     old_pose.v[2]) + sample->pose.v[2];
    double cs_bearing = cos(delta_bearing);
    double sn_bearing = sin(delta_bearing);

    // Sample just noise from pose differences
    delta_trans_hat = pf_ran_gaussian(trans_hat_stddev);
    delta_rot_hat = pf_ran_gaussian(rot_hat_stddev);
    delta_strafe_hat = pf_ran_gaussian(strafe_hat_stddev);
    // Apply sampled update to particle pose
    sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                          delta_strafe_hat * sn_bearing);
    sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                          delta_strafe_hat * cs_bearing);
    sample->pose.v[2] += delta_rot_hat;
  }
}

}  // namespace nav2_amcl
