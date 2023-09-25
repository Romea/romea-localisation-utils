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
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Pose2D & pose,
  romea_localisation_msgs::msg::ObservationPose2D & msg)
{
  to_ros_msg(pose, msg.pose);
  msg.level_arm.x = 0;
  msg.level_arm.y = 0;
  msg.level_arm.z = 0;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Pose2D & pose,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(pose, msg.observation_pose);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2D & msg)
{
  msg.pose.position.x = observation.Y(ObservationPose::POSITION_X);
  msg.pose.position.y = observation.Y(ObservationPose::POSITION_Y);
  msg.pose.yaw = observation.Y(ObservationPose::ORIENTATION_Z);
  msg.level_arm.x = observation.levelArm.x();
  msg.level_arm.y = observation.levelArm.y();
  msg.level_arm.z = observation.levelArm.z();

  for (size_t n = 0; n < 9; ++n) {
    msg.pose.covariance[n] = observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_pose);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationPose2DStamped & msg,
  ObservationPose & observation)
{
  observation.Y(ObservationPose::POSITION_X) = msg.observation_pose.pose.position.x;
  observation.Y(ObservationPose::POSITION_Y) = msg.observation_pose.pose.position.y;
  observation.Y(ObservationPose::ORIENTATION_Z) = msg.observation_pose.pose.yaw;
  observation.R() = Eigen::Matrix3d(msg.observation_pose.pose.covariance.data());
  observation.levelArm.x() = msg.observation_pose.level_arm.x;
  observation.levelArm.y() = msg.observation_pose.level_arm.y;
  observation.levelArm.z() = msg.observation_pose.level_arm.z;
}

}  // namespace romea
