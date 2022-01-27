#ifndef _romea_ObservationCourseConversions_hpp_
#define _romea_ObservationCourseConversions_hpp_

#include "romea_core_localisation/ObservationCourse.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_localisation_msgs/msg/observation_course_stamped.hpp>

namespace romea
{

void to_ros_msg(const ObservationCourse & observation,
              romea_localisation_msgs::msg::ObservationCourse &msg);

void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const ObservationCourse & observation,
              romea_localisation_msgs::msg::ObservationCourseStamped &msg);

void extractObs(const romea_localisation_msgs::msg::ObservationCourseStamped &msg,
                ObservationCourse & observation);

}

#endif
