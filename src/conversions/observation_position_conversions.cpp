// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// romea
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Position2D & position,
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
  const Position2D & position,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(position, msg.observation_position);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2D & msg)
{
  msg.position.x = observation.Y(ObservationPosition::POSITION_X);
  msg.position.y = observation.Y(ObservationPosition::POSITION_Y);
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
  const ObservationPosition & observation,
  romea_localisation_msgs::msg::ObservationPosition2DStamped & msg)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation, msg.observation_position);
}

//-----------------------------------------------------------------------------
void extract_obs(
  const romea_localisation_msgs::msg::ObservationPosition2DStamped & msg,
  ObservationPosition & observation)
{
  observation.Y(ObservationPosition::POSITION_X) = msg.observation_position.position.x;
  observation.Y(ObservationPosition::POSITION_Y) = msg.observation_position.position.y;
  observation.R() = Eigen::Matrix2d(msg.observation_position.position.covariance.data());
  observation.levelArm.x() = msg.observation_position.level_arm.x;
  observation.levelArm.y() = msg.observation_position.level_arm.y;
  observation.levelArm.z() = msg.observation_position.level_arm.z;
}

}  // namespace romea
