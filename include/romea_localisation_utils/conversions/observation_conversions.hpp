// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_CONVERSIONS_HPP_

#include "romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_course_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_linear_speed_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_linear_speeds_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_twist_conversions.hpp"

namespace romea
{

template<typename ObservationType, typename MessageType>
ObservationType extract_obs(const MessageType & msg)
{
  ObservationType observation;
  extract_obs(msg, observation);
  return observation;
}

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_CONVERSIONS_HPP_
