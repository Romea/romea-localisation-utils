#ifndef _romea_ObservationLinearSpeedsConversions_hpp_
#define _romea_ObservationLinearSpeedsConversions_hpp_

#include <romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp>
#include "romea_core_localisation/ObservationLinearSpeeds.hpp"

namespace romea
{

void extractObs(const romea_localisation_msgs::msg::ObservationTwist2DStamped &msg,
                ObservationLinearSpeeds & observation);

}

#endif
