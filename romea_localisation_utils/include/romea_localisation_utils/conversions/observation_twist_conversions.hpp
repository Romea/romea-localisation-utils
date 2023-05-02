// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_TWIST_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_TWIST_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp"
#include "romea_core_localisation/ObservationTwist.hpp"

namespace romea
{

void to_ros_msg(
  const Twist2D & twist,
  romea_localisation_msgs::msg::ObservationTwist2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Twist2D & twist,
  romea_localisation_msgs::msg::ObservationTwist2DStamped & msg);

void to_ros_msg(
  const ObservationTwist & observation,
  romea_localisation_msgs::msg::ObservationTwist2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationTwist & observation,
  romea_localisation_msgs::msg::ObservationTwist2DStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
  ObservationTwist & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_TWIST_CONVERSIONS_HPP_
