/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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

/* Author: Brian Gerkey */

#include "nav2_amcl/localization_score_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_amcl/portable_utils.hpp"

using namespace std::placeholders;
using rcl_interfaces::msg::ParameterType;
using namespace std::chrono_literals;

namespace nav2_amcl
{
using nav2_util::geometry_utils::orientationAroundZAxis;

LocalizationScoreNode::LocalizationScoreNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("localization_score", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")),
    "Which frame to use for the robot base");

  add_parameter("beam_skip_distance", rclcpp::ParameterValue(0.5));
  add_parameter("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  add_parameter("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  add_parameter("do_beamskip", rclcpp::ParameterValue(false));

  add_parameter(
    "global_frame_id", rclcpp::ParameterValue(std::string("map")),
    "The name of the coordinate frame published by the localization system");

  add_parameter(
    "lambda_short", rclcpp::ParameterValue(0.1),
    "Exponential decay parameter for z_short part of model");

  add_parameter(
    "laser_likelihood_max_dist", rclcpp::ParameterValue(2.0),
    "Maximum distance to do obstacle inflation on map, for use in likelihood_field model");

  add_parameter(
    "laser_max_range", rclcpp::ParameterValue(100.0),
    "Maximum scan range to be considered",
    "-1.0 will cause the laser's reported maximum range to be used");

  add_parameter(
    "laser_min_range", rclcpp::ParameterValue(-1.0),
    "Minimum scan range to be considered",
    "-1.0 will cause the laser's reported minimum range to be used");

  add_parameter(
    "laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")),
    "Which model to use, either beam, likelihood_field, or likelihood_field_prob",
    "Same as likelihood_field but incorporates the beamskip feature, if enabled");


  add_parameter(
    "max_beams", rclcpp::ParameterValue(60),
    "How many evenly-spaced beams in each scan to be used when updating the filter");

  add_parameter("sigma_hit", rclcpp::ParameterValue(0.2));

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(1.0),
    "Time with which to post-date the transform that is published, to indicate that this transform "
    "is valid into the future");

  add_parameter(
    "update_min_a", rclcpp::ParameterValue(0.2),
    "Rotational movement required before performing a filter update");

  add_parameter(
    "update_min_d", rclcpp::ParameterValue(0.25),
    "Translational movement required before performing a filter update");

  add_parameter("z_hit", rclcpp::ParameterValue(0.5));
  add_parameter("z_max", rclcpp::ParameterValue(0.05));
  add_parameter("z_rand", rclcpp::ParameterValue(0.5));
  add_parameter("z_short", rclcpp::ParameterValue(0.05));

  add_parameter(
    "scan_topic", rclcpp::ParameterValue("scan"),
    "Topic to subscribe to in order to receive the laser scan for localization");

  add_parameter(
    "map_topic", rclcpp::ParameterValue("map"),
    "Topic to subscribe to in order to receive the map to localize on");

  add_parameter(
    "first_map_only", rclcpp::ParameterValue(false),
    "Set this to true, when you want to load a new map published from the map_server");
}

LocalizationScoreNode::~LocalizationScoreNode()
{
}

nav2_util::CallbackReturn
LocalizationScoreNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  initParameters();
  initTransforms();
  initLaserScan();
  initMessageFilters();
  initPubSub();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationScoreNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Lifecycle publishers must be explicitly activated
  metrics_pub_->on_activate();
  effectively_used_laserscan_pub_->on_activate();
  likelihood_map_pub_->on_activate();

  // Keep track of whether we're in the active state. We won't
  // process incoming callbacks until we are
  active_ = true;

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &LocalizationScoreNode::dynamicParametersCallback,
      this, std::placeholders::_1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationScoreNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  active_ = false;

  // Lifecycle publishers must be explicitly deactivated
  metrics_pub_->on_deactivate();
  effectively_used_laserscan_pub_->on_deactivate();
  likelihood_map_pub_->on_deactivate();

  // reset dynamic parameter handler
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationScoreNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  executor_thread_.reset();

  // Get rid of the inputs first (services and message filter input), so we
  // don't continue to process incoming messages
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();

  // Map
  if (map_ != NULL) {
    map_free(map_);
    map_ = nullptr;
  }
  first_map_received_ = false;
  free_space_indices.resize(0);

  // Transforms
  tf_listener_.reset();
  tf_buffer_.reset();

  // PubSub
  metrics_pub_.reset();
  effectively_used_laserscan_pub_.reset();
  likelihood_map_pub_.reset();

  // Laser Scan
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationScoreNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool
LocalizationScoreNode::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{
  rclcpp::Duration elapsed_time = now() - last_time;
  if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
    return true;
  }
  return false;
}

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> LocalizationScoreNode::free_space_indices;
#endif

bool
LocalizationScoreNode::getGlobalPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  double & x, double & y, double & yaw,
  const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
{
  // Get the robot's pose
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
  ident.header.stamp = sensor_timestamp;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  try {
    tf_buffer_->transform(ident, global_pose, global_frame_id_);
  } catch (tf2::TransformException & e) {
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    return false;
  }

  scan_error_count_ = 0;  // reset since we got a good transform
  x = global_pose.pose.position.x;
  y = global_pose.pose.position.y;
  yaw = tf2::getYaw(global_pose.pose.orientation);

  return true;
}

pf_vector_t
LocalizationScoreNode::uniformPoseGenerator(void * arg)
{
  map_t * map = reinterpret_cast<map_t *>(arg);

#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  for (;; ) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}

void
LocalizationScoreNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    return;
  }

  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  int laser_index = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getGlobalPose(
      latest_global_pose_, pose.v[0], pose.v[1], pose.v[2],
      laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  if (!pf_init_) {
    // Pose at last filter update
    last_global_pose_ = pose;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }
  } else {
    // Set the laser update flags
    if (shouldComputeLocalizationScore(pose, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
  }

  // If the robot has moved, calculate the localization score
  if (lasers_update_[laser_index]) {
    calculateLocalizationScore(laser_index, laser_scan, pose);

    publishEffectivelyUsedLaserscan(laser_scan);
  }
}

bool LocalizationScoreNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  lasers_.push_back(createLaserObject());
  lasers_update_.push_back(true);
  laser_index = frame_to_laser_.size();

  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    tf_buffer_->transform(ident, laser_pose, base_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use: (%s)",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str(), e.what());
    return false;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;

  ///////////////////////////////////////////////////////////////////////////////////
  // compute the likelihood map
  RCLCPP_ERROR(get_logger(), "Start computing the likelihood map");
  map_t* map = lasers_[laser_index]->getMap();

  auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  map_msg->header.frame_id = global_frame_id_;

  map_msg->info.width = map->size_x;
  map_msg->info.height = map->size_y;
  map_msg->info.resolution = map->scale;
  map_msg->info.origin.position.x = map->origin_x - (map->size_x / 2) * map->scale;
  map_msg->info.origin.position.y = map->origin_y - (map->size_y / 2) * map->scale;
  map_msg->info.origin.orientation.w = 1.0;

  map_msg->data.reserve(map_msg->info.width * map_msg->info.height);


  double z_hit_denom = 2 * this->sigma_hit_ * this->sigma_hit_;
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    double z = map->cells[i].occ_dist;
    double pz = this->z_hit_ * exp(-(z * z) / z_hit_denom);

    pz *= 100;

    // RCLCPP_INFO(get_logger(), "z: %f Calculated likelihood %.15f%%", z, pz);

    if (pz > 100) {
      RCLCPP_INFO(
        get_logger(), "Calculated likelihood %.4f (z = %f) greater than maximum 100%%: Set to maximum value", pz, z);
      
      pz = 100;
    } else if (pz < 0) {
      RCLCPP_INFO(
        get_logger(), "Calculated likelihood %.4f (z = %f) smaller than minimum 0%%: Set to minimum value", pz, z);
      pz = 0;
    }
    map_msg->data.push_back(static_cast<int8_t>(std::round(pz)));
  }

  likelihood_map_pub_->publish(std::move(map_msg));  

  return true;
}

bool LocalizationScoreNode::shouldComputeLocalizationScore(const pf_vector_t pose, pf_vector_t & delta)
{
  delta.v[0] = pose.v[0] - last_global_pose_.v[0];
  delta.v[1] = pose.v[1] - last_global_pose_.v[1];
  delta.v[2] = angleutils::angle_diff(pose.v[2], last_global_pose_.v[2]);

  // See if we should update the filter
  bool update = fabs(delta.v[0]) > d_thresh_ ||
    fabs(delta.v[1]) > d_thresh_ ||
    fabs(delta.v[2]) > a_thresh_;

  return update;
}

bool LocalizationScoreNode::calculateLocalizationScore(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose)
{
  nav2_amcl::LaserData ldata;
  ldata.laser = lasers_[laser_index];
  ldata.range_count = laser_scan->ranges.size();
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  min_q.quaternion = orientationAroundZAxis(laser_scan->angle_min);

  inc_q.header = min_q.header;
  inc_q.quaternion = orientationAroundZAxis(laser_scan->angle_min + laser_scan->angle_increment);
  try {
    tf_buffer_->transform(min_q, min_q, base_frame_id_);
    tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(
      get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  if (laser_max_range_ > 0.0) {
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;
  }
  double range_min;
  if (laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;
  }

  // The LaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }

  auto fake_pf = pf_alloc(1, 1, 0.0, 0.0,
                          (pf_init_model_fn_t)LocalizationScoreNode::uniformPoseGenerator);
  
  fake_pf->sets[fake_pf->current_set].mean = pose;
  fake_pf->sets[fake_pf->current_set].sample_count = 1;
  fake_pf->sets[fake_pf->current_set].samples[0].pose = pose;
  fake_pf->sets[fake_pf->current_set].samples[0].weight = 1.0;

  auto sensor_model_score_ = lasers_[laser_index]->getSensorModelScore(fake_pf, reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
  RCLCPP_DEBUG(get_logger(), "Sensor model score: %f", sensor_model_score_);
  lasers_update_[laser_index] = false;
  last_global_pose_ = pose;

  pf_free(fake_pf);

  auto metrics = std::make_unique<nav2_msgs::msg::LocalizationMetrics>();
  metrics->header = laser_scan->header;
  metrics->max_weight = sensor_model_score_; // we just have one sample so we can just use the same value
  metrics->total_sensor_model_score = sensor_model_score_;
  metrics->number_of_particles = 1;

  metrics_pub_->publish(std::move(metrics));

  return true;
}

nav2_amcl::Laser *
LocalizationScoreNode::createLaserObject()
{
  RCLCPP_INFO(get_logger(), "createLaserObject");

  if (sensor_model_type_ == "beam") {
    return new nav2_amcl::BeamModel(
      z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
      0.0, max_beams_, map_);
  }

  if (sensor_model_type_ == "likelihood_field_prob") {
    return new nav2_amcl::LikelihoodFieldModelProb(
      z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
      beam_skip_error_threshold_, max_beams_, map_);
  }

  return new nav2_amcl::LikelihoodFieldModel(
    z_hit_, z_rand_, sigma_hit_,
    laser_likelihood_max_dist_, max_beams_, map_);
}

void
LocalizationScoreNode::initParameters()
{
  double tmp_tol;

  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("beam_skip_distance", beam_skip_distance_);
  get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  get_parameter("beam_skip_threshold", beam_skip_threshold_);
  get_parameter("do_beamskip", do_beamskip_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("lambda_short", lambda_short_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", laser_max_range_);
  get_parameter("laser_min_range", laser_min_range_);
  get_parameter("laser_model_type", sensor_model_type_);
  get_parameter("max_beams", max_beams_);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("transform_tolerance", tmp_tol);
  get_parameter("update_min_a", a_thresh_);
  get_parameter("update_min_d", d_thresh_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("z_short", z_short_);
  get_parameter("first_map_only", first_map_only_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("map_topic", map_topic_);

  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks
  if (laser_likelihood_max_dist_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set laser_likelihood_max_dist to be negtive,"
      " this isn't allowed so it will be set to default value 2.0.");
    laser_likelihood_max_dist_ = 2.0;
  }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
LocalizationScoreNode::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  double tmp_tol;

  bool reinit_laser = false;
  bool reinit_map = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "beam_skip_distance") {
        beam_skip_distance_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_error_threshold") {
        beam_skip_error_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_threshold") {
        beam_skip_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "lambda_short") {
        lambda_short_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_likelihood_max_dist") {
        laser_likelihood_max_dist_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_max_range") {
        laser_max_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_min_range") {
        laser_min_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "sigma_hit") {
        sigma_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "transform_tolerance") {
        tmp_tol = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(tmp_tol);
        reinit_laser = true;
      } else if (param_name == "update_min_a") {
        a_thresh_ = parameter.as_double();
      } else if (param_name == "update_min_d") {
        d_thresh_ = parameter.as_double();
      } else if (param_name == "z_hit") {
        z_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_max") {
        z_max_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_rand") {
        z_rand_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_short") {
        z_short_ = parameter.as_double();
        reinit_laser = true;
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame_id") {
        base_frame_id_ = parameter.as_string();
      } else if (param_name == "global_frame_id") {
        global_frame_id_ = parameter.as_string();
      } else if (param_name == "map_topic") {
        map_topic_ = parameter.as_string();
        reinit_map = true;
      } else if (param_name == "laser_model_type") {
        sensor_model_type_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "scan_topic") {
        scan_topic_ = parameter.as_string();
        reinit_laser = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "do_beamskip") {
        do_beamskip_ = parameter.as_bool();
        reinit_laser = true;
      } else if (param_name == "first_map_only") {
        first_map_only_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_beams") {
        max_beams_ = parameter.as_int();
        reinit_laser = true;
      }
    }
  }

  // Re-initialize the lasers and it's filters
  if (reinit_laser) {
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
    laser_scan_connection_.disconnect();
    laser_scan_sub_.reset();

    initMessageFilters();
  }

  // Re-initialize the map
  if (reinit_map) {
    map_sub_.reset();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&LocalizationScoreNode::mapReceived, this, std::placeholders::_1));
  }

  result.successful = true;
  return result;
}

void
LocalizationScoreNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "LocalizationScoreNode: A new map was received.");
  if (first_map_only_ && first_map_received_) {
    return;
  }
  handleMapMessage(*msg);
  first_map_received_ = true;
}

void
LocalizationScoreNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg.info.width,
    msg.info.height,
    msg.info.resolution);
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
      " cause issues with reading published topics",
      msg.header.frame_id.c_str(),
      global_frame_id_.c_str());
  }
  freeMapDependentMemory();
  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  createFreeSpaceVector();
#endif
}

void
LocalizationScoreNode::createFreeSpaceVector()
{
  // Index of free space
  free_space_indices.resize(0);
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }
}

void
LocalizationScoreNode::freeMapDependentMemory()
{
  if (map_ != NULL) {
    map_free(map_);
    map_ = NULL;
  }

  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
}

// Convert an OccupancyGrid map message into the internal representation. This function
// allocates a map_t and returns it.
map_t *
LocalizationScoreNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // Convert to player format
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

void
LocalizationScoreNode::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  latest_tf_valid_ = false;
  latest_tf_ = tf2::Transform::getIdentity();
}

void
LocalizationScoreNode::initMessageFilters()
{
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, sub_opt);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, global_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);


  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &LocalizationScoreNode::laserReceived,
      this, std::placeholders::_1));
}

void
LocalizationScoreNode::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  metrics_pub_ = create_publisher<nav2_msgs::msg::LocalizationMetrics>(
    "localization_metrics",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  effectively_used_laserscan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "effectively_used_laserscan",
    rclcpp::SensorDataQoS());

  likelihood_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "likelihood_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&LocalizationScoreNode::mapReceived, this, std::placeholders::_1));
    

  RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
}

void
LocalizationScoreNode::publishEffectivelyUsedLaserscan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  auto effectively_used_laserscan = std::make_unique<sensor_msgs::msg::LaserScan>();
  effectively_used_laserscan->header = laser_scan->header;

  auto step = (laser_scan->ranges.size() - 1) / (max_beams_ - 1);

  // Step size must be at least 1
  if (step < 1) {
    step = 1;
  }

  auto number_ranges = laser_scan->ranges.size() / step;

  effectively_used_laserscan->angle_min = laser_scan->angle_min;
  effectively_used_laserscan->angle_increment = laser_scan->angle_increment * step;
  effectively_used_laserscan->angle_max = effectively_used_laserscan->angle_min + number_ranges * effectively_used_laserscan->angle_increment;
  effectively_used_laserscan->time_increment = laser_scan->time_increment * step;
  effectively_used_laserscan->scan_time = laser_scan->scan_time;

  effectively_used_laserscan->range_min = laser_scan->range_min;
  effectively_used_laserscan->range_max = laser_scan->range_max;

  effectively_used_laserscan->ranges.reserve(number_ranges);

  if (!laser_scan->intensities.empty()) {
    effectively_used_laserscan->intensities.reserve(number_ranges);
  }

  for (size_t i = 0; i < laser_scan->ranges.size(); i += step) {
    effectively_used_laserscan->ranges.push_back(laser_scan->ranges[i]);

    if (!laser_scan->intensities.empty()) {
      effectively_used_laserscan->intensities.push_back(laser_scan->intensities[i]);
    }
  }

  effectively_used_laserscan_pub_->publish(std::move(effectively_used_laserscan));  
}

void
LocalizationScoreNode::initLaserScan()
{
  scan_error_count_ = 0;
  last_laser_received_ts_ = rclcpp::Time(0);
}

}  // namespace nav2_amcl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_amcl::LocalizationScoreNode)
