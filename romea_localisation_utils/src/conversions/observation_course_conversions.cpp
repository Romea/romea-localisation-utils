#include "romea_localisation_utils/conversions/observation_course_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const ObservationCourse & observation,
                romea_localisation_msgs::msg::ObservationCourse &msg)
{
  msg.angle = observation.Y();
  msg.std = std::sqrt(observation.R());
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationCourse & observation,
                romea_localisation_msgs::msg::ObservationCourseStamped &msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  to_ros_msg(observation,msg.observation_course);
}

//-----------------------------------------------------------------------------
void extract_obs(const romea_localisation_msgs::msg::ObservationCourseStamped &msg,
                 ObservationCourse &observation)
{
  observation.Y()=msg.observation_course.angle;
  observation.R()=msg.observation_course.std*msg.observation_course.std;
}


}
