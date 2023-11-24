// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

// std
#include <string>

// romea
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Position2D & position,
  romea_localisation_msgs::msg::ObservationPosition2D & msg)
{
  to_ros_msg(position, msg.position);
  msg.level_arm.x = 0;
  msg.level_arm.y = 0;
  msg.level_arm.z = 0;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Position2D & position,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(position, msg.observation_position);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2D & msg)
{
  msg.position.x = observation.Y(core::ObservationPosition::POSITION_X);
  msg.position.y = observation.Y(core::ObservationPosition::POSITION_Y);
  msg.level_arm.x = observation.levelArm.x();
  msg.level_arm.y = observation.levelArm.y();
  msg.level_arm.z = observation.levelArm.z();

  for (size_t n = 0; n < 4; ++n) {
    msg.position.covariance[n] = observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_position);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationPosition2DStamped & msg,
  core::ObservationPosition & observation)
{
  observation.Y(core::ObservationPosition::POSITION_X) = msg.observation_position.position.x;
  observation.Y(core::ObservationPosition::POSITION_Y) = msg.observation_position.position.y;
  observation.R() = Eigen::Matrix2d(msg.observation_position.position.covariance.data());
  observation.levelArm.x() = msg.observation_position.level_arm.x;
  observation.levelArm.y() = msg.observation_position.level_arm.y;
  observation.levelArm.z() = msg.observation_position.level_arm.z;
}

}  // namespace ros2
}  // namespace romea
