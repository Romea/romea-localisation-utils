#ifndef _romea_LocalisationStatusConversions_hpp_
#define _romea_LocalisationStatusConversions_hpp_

#include "romea_core_localisation/LocalisationFSMState.hpp"
#include <romea_localisation_msgs/msg/localisation_status.hpp>


namespace romea {


  void to_ros_msg(const LocalisationFSMState & fsm_state,
                  romea_localisation_msgs::msg::LocalisationStatus &msg);

}

#endif
