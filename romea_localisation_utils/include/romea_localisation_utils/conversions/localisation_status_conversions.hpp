// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__CONVERSIONS__LOCALISATION_STATUS_CONVERSIONS_HPP_
#define ROMEA_LOCALISATION_UTILS__CONVERSIONS__LOCALISATION_STATUS_CONVERSIONS_HPP_

#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_localisation_msgs/msg/localisation_status.hpp"


namespace romea
{

void to_ros_msg(
  const LocalisationFSMState & fsm_state,
  romea_localisation_msgs::msg::LocalisationStatus & msg);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__CONVERSIONS__LOCALISATION_STATUS_CONVERSIONS_HPP_
