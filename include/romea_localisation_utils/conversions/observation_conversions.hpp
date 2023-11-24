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
namespace ros2
{

template<typename ObservationType, typename MessageType>
ObservationType extract_obs(const MessageType & msg)
{
  ObservationType observation;
  extract_obs(msg, observation);
  return observation;
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__OBSERVATION_CONVERSIONS_HPP_
