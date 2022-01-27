// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsAngularSpeedConversion : public ::testing::Test
{
public :

  TestObsAngularSpeedConversion():
    stamp(1000),
    frame_id("foo"),
    romea_obs_angular_speed(),
    ros_obs_angular_speed_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_obs_angular_speed.Y()=1;
    romea_obs_angular_speed.R()=4;
    romea::to_ros_msg(stamp,frame_id,romea_obs_angular_speed,ros_obs_angular_speed_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationAngularSpeed romea_obs_angular_speed;
  romea_localisation_msgs::msg::ObservationAngularSpeedStamped ros_obs_angular_speed_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsAngularSpeedConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_obs_angular_speed_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(ros_obs_angular_speed_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_obs_angular_speed_msg.observation_angular_speed.velocity,romea_obs_angular_speed.Y());
  EXPECT_DOUBLE_EQ(ros_obs_angular_speed_msg.observation_angular_speed.std,std::sqrt(romea_obs_angular_speed.R()));
}

//-----------------------------------------------------------------------------
TEST_F(TestObsAngularSpeedConversion, fromRosMsgtoObs)
{
  romea::ObservationAngularSpeed romea_obs_angular_speed_bis;
  romea::extractObs(ros_obs_angular_speed_msg,romea_obs_angular_speed_bis);

  EXPECT_DOUBLE_EQ(romea_obs_angular_speed_bis.Y(),romea_obs_angular_speed.Y());
  EXPECT_DOUBLE_EQ(romea_obs_angular_speed_bis.R(),romea_obs_angular_speed.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
