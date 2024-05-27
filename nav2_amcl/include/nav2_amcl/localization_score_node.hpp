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

#ifndef NAV2_AMCL__LOCALIZATION_SCORE_NODE_HPP_
#define NAV2_AMCL__LOCALIZATION_SCORE_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/msg/localization_metrics.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop

#define NEW_UNIFORM_SAMPLING 1

namespace nav2_amcl
{
/*
 * @class LocalizationScoreNode
 * @brief ROS wrapper for AMCL
 */
class LocalizationScoreNode : public nav2_util::LifecycleNode
{
public:
  /*
   * @brief AMCL constructor
   * @param options Additional options to control creation of the node.
   */
  explicit LocalizationScoreNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /*
   * @brief AMCL destructor
   */
  ~LocalizationScoreNode();

protected:
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Since the sensor data from gazebo or the robot is not lifecycle enabled, we won't
  // respond until we're in the active state
  std::atomic<bool> active_{false};

  // Dedicated callback group and executor for services and subscriptions in LocalizationScoreNode,
  // in order to isolate TF timer used in message filter.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

  // Pose hypothesis
  typedef struct
  {
    double weight;             // Total weight (weights sum to 1)
    pf_vector_t pf_pose_mean;  // Mean of pose esimate
    pf_matrix_t pf_pose_cov;   // Covariance of pose estimate
  } amcl_hyp_t;

  // Map-related
  /*
   * @brief Get new map from ROS topic to localize in
   * @param msg Map message
   */
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /*
   * @brief Handle a new map message
   * @param msg Map message
   */
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  /*
   * @brief Creates lookup table of free cells in map
   */
  void createFreeSpaceVector();
  /*
   * @brief Frees allocated map related memory
   */
  void freeMapDependentMemory();
  map_t * map_{nullptr};
  /*
   * @brief Convert an occupancy grid map to an AMCL map
   * @param map_msg Map message
   * @return pointer to map for AMCL to use
   */
  map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  bool first_map_only_{true};
  std::atomic<bool> first_map_received_{false};
  std::recursive_mutex mutex_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices;
#endif

  // Transforms
  /*
   * @brief Initialize required ROS transformations
   */
  void initTransforms();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool latest_tf_valid_{false};
  tf2::Transform latest_tf_;

  // Message filters
  /*
   * @brief Initialize incoming data message subscribers and filters
   */
  void initMessageFilters();
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp_lifecycle::LifecycleNode>> laser_scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  // Publishers and subscribers
  /*
   * @brief Initialize pub subs of AMCL
   */
  void initPubSub();
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::LocalizationMetrics>::SharedPtr
    metrics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr
    effectively_used_laserscan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    likelihood_map_pub_;
  
  void publishEffectivelyUsedLaserscan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);

  /*
   * @brief Handle when a laser scan is received
   */
  void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);

  geometry_msgs::msg::PoseStamped latest_global_pose_;
  
  /*
   * @brief Get robot pose in global frame using TF
   */
  bool getGlobalPose(
    // Helper to get global pose from transform system
    geometry_msgs::msg::PoseStamped & pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & sensor_timestamp, const std::string & frame_id);

  /*
   * @brief Pose-generating function used to uniformly distribute particles over the map
   */
  static pf_vector_t uniformPoseGenerator(void * arg);
  pf_t * pf_{nullptr};
  bool pf_init_;
  pf_vector_t last_global_pose_;
  // double tolal_sensor_model_score_;
  // int resample_count_{0};

  // Laser scan related
  /*
   * @brief Initialize laser scan
   */
  void initLaserScan();
  /*
   * @brief Create a laser object
   */
  nav2_amcl::Laser * createLaserObject();
  int scan_error_count_{0};
  std::vector<nav2_amcl::Laser *> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;
  rclcpp::Time last_laser_received_ts_;

  /*
   * @brief Check if sufficient time has elapsed to get an update
   */
  bool checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time);
  rclcpp::Time last_time_printed_msg_;
  /*
   * @brief Add a new laser scanner if a new one is received in the laser scallbacks
   */
  bool addNewScanner(
    int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::string & laser_scan_frame_id,
    geometry_msgs::msg::PoseStamped & laser_pose);
  /*
   * @brief Whether the pf needs to be updated
   */
  bool shouldComputeLocalizationScore(const pf_vector_t pose, pf_vector_t & delta);
  /*
   * @brief Update the PF
   */
  bool calculateLocalizationScore(
    const int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const pf_vector_t & pose);

  /*
   * @brief Get ROS parameters for node
   */
  void initParameters();
  std::string base_frame_id_;
  double beam_skip_distance_;
  double beam_skip_error_threshold_;
  double beam_skip_threshold_;
  bool do_beamskip_;
  std::string global_frame_id_;
  double lambda_short_;
  double laser_likelihood_max_dist_;
  double laser_max_range_;
  double laser_min_range_;
  std::string sensor_model_type_;
  int max_beams_;
  double sigma_hit_;
  tf2::Duration transform_tolerance_;
  double a_thresh_;
  double d_thresh_;
  double z_hit_;
  double z_max_;
  double z_short_;
  double z_rand_;
  std::string scan_topic_{"scan"};
  std::string map_topic_{"map"};
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__LOCALIZATION_SCORE_NODE_HPP_
