#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const ObservationRange & observation,
              romea_localisation_msgs::msg::ObservationRange &msg)
{
  msg.range = observation.Y();
  msg.range_std = std::sqrt(observation.R());
  msg.initiator_antenna_position.x = observation.initiatorPosition.x();
  msg.initiator_antenna_position.y = observation.initiatorPosition.y();
  msg.initiator_antenna_position.z = observation.initiatorPosition.z();
  msg.responder_antenna_position.x = observation.responderPosition.x();
  msg.responder_antenna_position.y = observation.responderPosition.y();
  msg.responder_antenna_position.z = observation.responderPosition.z();
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
              const std::string & frame_id,
              const ObservationRange & observation,
              romea_localisation_msgs::msg::ObservationRangeStamped &msg)
{
  msg.header.frame_id=frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation,msg.observation_range);
}

//-----------------------------------------------------------------------------
void extractObs(const romea_localisation_msgs::msg::ObservationRangeStamped &msg,
                ObservationRange &observation)
{
    observation.Y()= msg.observation_range.range;
    observation.R()= msg.observation_range.range_std*msg.observation_range.range_std;
    observation.initiatorPosition.x()=msg.observation_range.initiator_antenna_position.x;
    observation.initiatorPosition.y()=msg.observation_range.initiator_antenna_position.y;
    observation.initiatorPosition.z()=msg.observation_range.initiator_antenna_position.z;
    observation.responderPosition.x()=msg.observation_range.responder_antenna_position.x;
    observation.responderPosition.y()=msg.observation_range.responder_antenna_position.y;
    observation.responderPosition.z()=msg.observation_range.responder_antenna_position.z;
}

}
