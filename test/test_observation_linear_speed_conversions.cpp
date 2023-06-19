// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_linear_speed_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsLinearSpeedConversion : public ::testing::Test
{
public:
  TestObsLinearSpeedConversion()
  : romea_obs_linear_speed(),
    ros_obs_linear_speed_msg()
  {
  }

  void SetUp()override
  {
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.x = 1;
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.y = 2;
    ros_obs_linear_speed_msg.observation_twist.twist.angular_speed = 3;
    ros_obs_linear_speed_msg.observation_twist.level_arm.x = 4;
    ros_obs_linear_speed_msg.observation_twist.level_arm.x = 5;
    ros_obs_linear_speed_msg.observation_twist.level_arm.x = 6;
    fillMsgCovariance(ros_obs_linear_speed_msg.observation_twist.twist.covariance);
  }

  romea::ObservationLinearSpeed romea_obs_linear_speed;
  romea_localisation_msgs::msg::ObservationTwist2DStamped ros_obs_linear_speed_msg;
};


//-----------------------------------------------------------------------------
TEST_F(TestObsLinearSpeedConversion, fromRosMsgtoObs)
{
  romea::ObservationLinearSpeed romea_obs_linear_speed;
  romea::extract_obs(ros_obs_linear_speed_msg, romea_obs_linear_speed);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speed.Y(),
    ros_obs_linear_speed_msg.observation_twist.twist.linear_speeds.x);
  EXPECT_DOUBLE_EQ(
    romea_obs_linear_speed.R(),
    ros_obs_linear_speed_msg.observation_twist.twist.covariance[0]);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
