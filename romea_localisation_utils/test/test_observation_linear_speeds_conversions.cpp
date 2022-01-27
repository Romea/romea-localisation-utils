// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_linear_speeds_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsLinearSpeedsConversion : public ::testing::Test
{
public :

  TestObsLinearSpeedsConversion():
    romea_obs_linear_speeds(),
    ros_obs_linear_speeds_msg()
  {
  }

  virtual void SetUp()override
  {
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.x=1;
    ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.y=2;
    ros_obs_linear_speeds_msg.observation_twist.twist.angular_speed=3;
    ros_obs_linear_speeds_msg.observation_twist.level_arm.x=4;
    ros_obs_linear_speeds_msg.observation_twist.level_arm.x=5;
    ros_obs_linear_speeds_msg.observation_twist.level_arm.x=6;
    fillMsgCovariance(ros_obs_linear_speeds_msg.observation_twist.twist.covariance);
  }

  romea::ObservationLinearSpeeds romea_obs_linear_speeds;
  romea_localisation_msgs::msg::ObservationTwist2DStamped ros_obs_linear_speeds_msg;
};


//-----------------------------------------------------------------------------
TEST_F(TestObsLinearSpeedsConversion, fromRosMsgtoObs)
{
  romea::ObservationLinearSpeeds romea_obs_linear_speeds;
  romea::extractObs(ros_obs_linear_speeds_msg,romea_obs_linear_speeds);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.Y(romea::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.x);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.Y(romea::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.linear_speeds.y);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.R(romea::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
                                             romea::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.covariance[0]);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.R(romea::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
                                             romea::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.covariance[1]);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.R(romea::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
                                             romea::ObservationLinearSpeeds::LINEAR_SPEED_X_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.covariance[3]);
  EXPECT_DOUBLE_EQ(romea_obs_linear_speeds.R(romea::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
                                             romea::ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY),
                   ros_obs_linear_speeds_msg.observation_twist.twist.covariance[4]);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
