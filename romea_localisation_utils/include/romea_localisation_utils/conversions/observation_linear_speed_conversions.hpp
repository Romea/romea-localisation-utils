#ifndef _romea_ObservationLinearSpeedConversions_hpp_
#define _romea_ObservationLinearSpeedConversions_hpp_

#include <romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp>
#include "romea_core_localisation/ObservationLinearSpeed.hpp"

namespace romea
{

void extractObs(const romea_localisation_msgs::msg::ObservationTwist2DStamped &msg,
                ObservationLinearSpeed & observation);

}

#endif
