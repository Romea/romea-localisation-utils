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

#include "romea_localisation_utils/conversions/localisation_status_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::LocalisationFSMState & fsm_state,
  romea_localisation_msgs::msg::LocalisationStatus & msg)
{
  msg.status = static_cast<unsigned int>(fsm_state);
}

}  // namespace ros2
}  // namespace romea
