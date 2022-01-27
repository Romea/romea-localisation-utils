#ifndef _romea_ObservationConversions_hpp_
#define _romea_ObservationConversions_hpp

#include "observation_angular_speed_conversions.hpp"
#include "observation_attitude_conversions.hpp"
#include "observation_course_conversions.hpp"
#include "observation_linear_speed_conversions.hpp"
#include "observation_linear_speeds_conversions.hpp"
#include "observation_pose_conversions.hpp"
#include "observation_position_conversions.hpp"
#include "observation_range_conversions.hpp"
#include "observation_twist_conversions.hpp"

namespace romea
{

template <typename ObservationType, typename MessageType>
ObservationType extractObs(const MessageType & msg)
{
  ObservationType observation;
  extractObs(msg,observation);
  return observation;
}


}

#endif
