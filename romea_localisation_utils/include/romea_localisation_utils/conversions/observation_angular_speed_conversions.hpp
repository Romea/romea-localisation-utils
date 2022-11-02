#ifndef _romea_ObservationAngularSpeedConversions_hpp_
#define _romea_ObservationAngularSpeedConversions_hpp_

#include "romea_core_localisation/ObservationAngularSpeed.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_localisation_msgs/msg/observation_angular_speed_stamped.hpp>


namespace romea {


void to_ros_msg(const ObservationAngularSpeed & observation,
                romea_localisation_msgs::msg::ObservationAngularSpeed &msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationAngularSpeed & observation,
                romea_localisation_msgs::msg::ObservationAngularSpeedStamped &msg);

void extract_obs(const romea_localisation_msgs::msg::ObservationAngularSpeedStamped & msg,
                 ObservationAngularSpeed & observation);
}

#endif
