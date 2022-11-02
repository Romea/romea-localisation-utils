#include "romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const ObservationAngularSpeed & observation,
                romea_localisation_msgs::msg::ObservationAngularSpeed &msg)
{
  msg.velocity = observation.Y();
  msg.std = std::sqrt(observation.R());
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationAngularSpeed & observation,
                romea_localisation_msgs::msg::ObservationAngularSpeedStamped &msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  to_ros_msg(observation,msg.observation_angular_speed);
}

//-----------------------------------------------------------------------------
void extract_obs(const romea_localisation_msgs::msg::ObservationAngularSpeedStamped & msg,
                 ObservationAngularSpeed &observation)
{
  observation.Y() =msg.observation_angular_speed.velocity;
  observation.R() =msg.observation_angular_speed.std*msg.observation_angular_speed.std;
}


}
