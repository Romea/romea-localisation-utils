// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEEDS_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEEDS_CONVERSIONS_HPP_

#include "romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp"
#include "romea_core_localisation/ObservationLinearSpeeds.hpp"

namespace romea
{

void extract_obs(
  const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
  ObservationLinearSpeeds & observation);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_LINEAR_SPEEDS_CONVERSIONS_HPP_
