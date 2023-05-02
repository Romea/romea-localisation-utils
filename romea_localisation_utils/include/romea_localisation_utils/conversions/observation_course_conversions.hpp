// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_core_localisation/ObservationCourse.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_localisation_msgs/msg/observation_course_stamped.hpp"

namespace romea
{

void to_ros_msg(
  const ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourse & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const ObservationCourse & observation,
  romea_localisation_msgs::msg::ObservationCourseStamped & msg);

void extract_obs(
  const romea_localisation_msgs::msg::ObservationCourseStamped & msg,
  ObservationCourse & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_COURSE_CONVERSIONS_HPP_
