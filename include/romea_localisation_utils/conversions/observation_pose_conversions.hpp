// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSE_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSE_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include "romea_localisation_msgs/msg/observation_pose2_d_stamped.hpp"
#include "romea_core_localisation/ObservationPose.hpp"


namespace romea
{

void to_ros_msg(
  const Pose2D & pose,
  romea_localisation_msgs::msg::ObservationPose2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Pose2D & position,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg);

void to_ros_msg(
  const ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationPose & observation,
  romea_localisation_msgs::msg::ObservationPose2DStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationPose2DStamped & msg,
  ObservationPose & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSE_CONVERSIONS_HPP_
