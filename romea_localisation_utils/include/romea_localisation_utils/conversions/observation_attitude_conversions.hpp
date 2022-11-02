#ifndef _romea_ObservationAttitudeConversions_hpp_
#define _romea_ObservationAttitudeConversions_hpp_

#include "romea_core_localisation/ObservationAttitude.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_localisation_msgs/msg/observation_attitude_stamped.hpp>

namespace romea
{

void to_ros_msg(const ObservationAttitude & observation,
                romea_localisation_msgs::msg::ObservationAttitude &msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationAttitude & observation,
                romea_localisation_msgs::msg::ObservationAttitudeStamped &msg);

void extract_obs(const romea_localisation_msgs::msg::ObservationAttitudeStamped &msg,
                 ObservationAttitude & observation);

}

#endif
