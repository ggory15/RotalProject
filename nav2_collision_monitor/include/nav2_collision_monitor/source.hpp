// Copyright (c) 2022 Samsung Research Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_COLLISION_MONITOR__SOURCE_HPP_
#define NAV2_COLLISION_MONITOR__SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic data source class
 */
class Source
{
public:
  /**
   * @brief Source constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  Source(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const std::string & node_name);
  /**
   * @brief Source destructor
   */
  virtual ~Source();

  /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  virtual void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const = 0;

protected:
  /**
   * @brief Supporting routine obtaining ROS-parameters common for all data sources
   * @param source_topic Output name of source subscription topic
   */
  void getCommonParameters(std::string & source_topic);

  /**
   * @brief Checks whether the source data might be considered as valid
   * @param source_time Timestamp of latest obtained data
   * @param curr_time Current node time for source verification
   * @return True if data source is valid, otherwise false
   */
  bool sourceValid(
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time) const;

  /**
   * @brief Obtains a transform from source_frame_id at source_time ->
   * to base_frame_id_ at curr_time time
   * @param source_frame_id Source frame ID to convert data from
   * @param source_time Source timestamp to convert data from
   * @param curr_time Current node time to interpolate data to
   * @param tf_transform Output source->base transform
   * @return True if got correct transform, otherwise false
   */
  bool getTransform(
    const std::string & source_frame_id,
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time,
    tf2::Transform & tf_transform) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Name of data source
  std::string source_name_;

  // Global variables
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Robot base frame ID
  std::string base_frame_id_;
  /// @brief Global frame ID for correct transform calculation
  std::string global_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Maximum time interval in which data is considered valid
  rclcpp::Duration source_timeout_;

  std::string node_name_;
};  // class Source

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_HPP_
