#include "romea_localisation_utils/conversions/observation_linear_speed_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void extract_obs(const romea_localisation_msgs::msg::ObservationTwist2DStamped &msg,
                 ObservationLinearSpeed &observation)
{
  observation.Y()=msg.observation_twist.twist.linear_speeds.x;
  observation.R()=msg.observation_twist.twist.covariance[0];
}

}
