#include "romea_localisation_utils/conversions/observation_twist_conversions.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
void to_ros_msg(const Twist2D & twist,
                romea_localisation_msgs::msg::ObservationTwist2D &msg)
{
  to_ros_msg(twist,msg.twist);
  msg.level_arm.x=0;
  msg.level_arm.y=0;
  msg.level_arm.z=0;

}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const Twist2D & twist,
                romea_localisation_msgs::msg::ObservationTwist2DStamped &msg)
{
  msg.header.frame_id=frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(twist,msg.observation_twist);
}

//-----------------------------------------------------------------------------
void to_ros_msg(const ObservationTwist & observation,
                romea_localisation_msgs::msg::ObservationTwist2D &msg)
{

  msg.twist.linear_speeds.x=observation.Y(ObservationTwist::LINEAR_SPEED_X_BODY);
  msg.twist.linear_speeds.y=observation.Y(ObservationTwist::LINEAR_SPEED_Y_BODY);
  msg.twist.angular_speed=observation.Y(ObservationTwist::ANGULAR_SPEED_Z_BODY);
  msg.level_arm.x=observation.levelArm.x();
  msg.level_arm.y=observation.levelArm.y();
  msg.level_arm.z=observation.levelArm.z();

  for(size_t n=0;n<9;++n)
  {
    msg.twist.covariance[n]=observation.R()(n);
  }

}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const ObservationTwist & observation,
                romea_localisation_msgs::msg::ObservationTwist2DStamped &msg)
{
  msg.header.frame_id=frame_id;
  msg.header.stamp = stamp;
  to_ros_msg(observation,msg.observation_twist);
}

//-----------------------------------------------------------------------------
void extract_obs(const romea_localisation_msgs::msg::ObservationTwist2DStamped & msg,
                 ObservationTwist &observation)
{
  observation.Y(ObservationTwist::LINEAR_SPEED_X_BODY)=msg.observation_twist.twist.linear_speeds.x;
  observation.Y(ObservationTwist::LINEAR_SPEED_Y_BODY)=msg.observation_twist.twist.linear_speeds.y;
  observation.Y(ObservationTwist::ANGULAR_SPEED_Z_BODY)=msg.observation_twist.twist.angular_speed;
  observation.R()=Eigen::Matrix3d(msg.observation_twist.twist.covariance.data());
  observation.levelArm.x()=msg.observation_twist.level_arm.x;
  observation.levelArm.y()=msg.observation_twist.level_arm.y;
  observation.levelArm.z()=msg.observation_twist.level_arm.z;
}

}
