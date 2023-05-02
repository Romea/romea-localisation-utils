// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSITION_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSITION_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_common_utils/conversions/position2d_conversions.hpp"
#include "romea_localisation_msgs/msg/observation_position2_d_stamped.hpp"
#include "romea_core_localisation/ObservationPosition.hpp"

namespace romea
{

void to_ros_msg(
  const Position2D & position,
  romea_localisation_msgs::msg::ObservationPosition2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Position2D & position,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg);


void to_ros_msg(
  const ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2D & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationPosition2DStamped & msg,
  ObservationPosition & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_POSITION_CONVERSIONS_HPP_
