#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const ObservationAttitude & observation,
              romea_localisation_msgs::msg::ObservationAttitude &msg)
{
  msg.roll_angle = observation.Y(romea::ObservationAttitude::ROLL);
  msg.pitch_angle = observation.Y(romea::ObservationAttitude::PITCH);

  for(size_t n=0;n<4;++n)
  {
    msg.covariance[n]=observation.R()(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const ObservationAttitude & observation,
              romea_localisation_msgs::msg::ObservationAttitudeStamped &msg)
{
  msg.header.frame_id=frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation,msg.observation_attitude);
}


//-----------------------------------------------------------------------------
void extractObs(const romea_localisation_msgs::msg::ObservationAttitudeStamped &msg,
                ObservationAttitude & observation)
{
  observation.Y(ObservationAttitude::ROLL)=msg.observation_attitude.roll_angle;
  observation.Y(ObservationAttitude::PITCH)=msg.observation_attitude.pitch_angle;
  observation.R() = Eigen::Matrix2d(msg.observation_attitude.covariance.data());
}


}
