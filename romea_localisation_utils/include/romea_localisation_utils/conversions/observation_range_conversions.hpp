#ifndef _romea_ObservationRangeConversions_hpp_
#define _romea_ObservationRangeConversions_hpp_

#include "romea_core_localisation/ObservationRange.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_localisation_msgs/msg/observation_range_stamped.hpp>

namespace romea
{

void to_ros_msg(const ObservationRange & observation,
              romea_localisation_msgs::msg::ObservationRange &msg);

void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const ObservationRange &observation,
              romea_localisation_msgs::msg::ObservationRangeStamped &msg);

void extractObs(const romea_localisation_msgs::msg::ObservationRangeStamped & msg,
                ObservationRange & observation);

}

#endif
