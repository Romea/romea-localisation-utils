#ifndef _romea_PositionConversions_hpp_
#define _romea_PositionConversions_hpp_

#include "romea_common_utils/conversions/position2d_conversions.hpp"
#include <romea_localisation_msgs/msg/observation_position2_d_stamped.hpp>
#include "romea_core_localisation/ObservationPosition.hpp"

namespace romea
{

void to_ros_msg(const Position2D & position,
                romea_localisation_msgs::msg::ObservationPosition2D &msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const Position2D & position,
                romea_localisation_msgs::msg::ObservationPosition2DStamped &msg);


void to_ros_msg(const ObservationPosition & observation,
                romea_localisation_msgs::msg::ObservationPosition2D &msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationPosition & observation,
                romea_localisation_msgs::msg::ObservationPosition2DStamped &msg);

void extract_obs(const romea_localisation_msgs::msg::ObservationPosition2DStamped & msg,
                 ObservationPosition & observation);

}

#endif
