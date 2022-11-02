#include "romea_localisation_utils/conversions/observation_linear_speeds_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void extract_obs(const romea_localisation_msgs::msg::ObservationTwist2DStamped &msg,
                 ObservationLinearSpeeds &observation)
{
  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY)=
      msg.observation_twist.twist.linear_speeds.x;
  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY)=
      msg.observation_twist.twist.linear_speeds.y;
  observation.R(0,0) = msg.observation_twist.twist.covariance[0],
      observation.R(0,1) = msg.observation_twist.twist.covariance[1];
  observation.R(1,0) = msg.observation_twist.twist.covariance[3],
      observation.R(1,1) = msg.observation_twist.twist.covariance[4];
}

}
