// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_localisation_utils/conversions/localisation_status_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const LocalisationFSMState & fsm_state,
  romea_localisation_msgs::msg::LocalisationStatus & msg)
{
  msg.status = static_cast<unsigned int>(fsm_state);
}

}  // namespace romea
