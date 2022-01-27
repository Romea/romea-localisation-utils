#include "romea_localisation_utils/conversions/observation_conversions.hpp"

namespace romea
{



////-----------------------------------------------------------------------------
//void extractObs(const romea_localisation_msgs::Twist2DStamped &msg,
//                ObservationTwist &observation)
//{
//  observation.Y(ObservationTwist::LINEAR_SPEED_X_BODY)=msg.twist.linear_speed_along_x_body_axis;
//  observation.Y(ObservationTwist::LINEAR_SPEED_Y_BODY)=msg.twist.linear_speed_along_y_body_axis;
//  observation.Y(ObservationTwist::ANGULAR_SPEED_Z_BODY)=msg.twist.angular_speed_around_z_body_axis;
//  observation.R()=Eigen::Matrix3d(msg.twist.covariance.data());
//}

////-----------------------------------------------------------------------------
//void extractObs(const romea_localisation_msgs::Pose2DStamped & msg,
//                ObservationPose & observation)
//{
//  observation.Y(ObservationPose::POSITION_X)=msg.pose.x;
//  observation.Y(ObservationPose::POSITION_Y)=msg.pose.y;
//  observation.Y(ObservationPose::ORIENTATION_Z)=msg.pose.yaw;
//  observation.R() = Eigen::Matrix3d(msg.pose.covariance.data());
//  observation.levelArm << msg.pose.level_arm.x, msg.pose.level_arm.y, msg.pose.level_arm.z;
//}



////-----------------------------------------------------------------------------
//void extractObs(const romea_localisation_msgs::RangeStamped & msg,
//                ObservationRange &observation)
//{
//  observation.Y()= msg.range.range;
//  observation.R()= msg.range.range_std*msg.range.range_std;
//  observation.initiatorPosition.x()=msg.range.initiator_antenna_position.x;
//  observation.initiatorPosition.y()=msg.range.initiator_antenna_position.y;
//  observation.initiatorPosition.z()=msg.range.initiator_antenna_position.z;
//  observation.responderPosition.x()=msg.range.responder_antenna_position.x;
//  observation.responderPosition.y()=msg.range.responder_antenna_position.y;
//  observation.responderPosition.z()=msg.range.responder_antenna_position.z;
//}

////-----------------------------------------------------------------------------
//void extractObs(const romea_localisation_msgs::Twist2DStamped &msg,
//                ObservationLinearSpeed &observation)
//{
//  observation.Y()=msg.twist.linear_speed_along_x_body_axis;
//  observation.R()=msg.twist.covariance[0];
//}

////-----------------------------------------------------------------------------
//void extractObs(const romea_localisation_msgs::Twist2DStamped &msg,
//                ObservationLinearSpeeds &observation)
//{
//  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY)=
//      msg.twist.linear_speed_along_x_body_axis;
//  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY)=
//      msg.twist.linear_speed_along_y_body_axis;
//  observation.R().row(0)<<msg.twist.covariance[0],msg.twist.covariance[1];
//  observation.R().row(1)<<msg.twist.covariance[3],msg.twist.covariance[4];
//}

}
