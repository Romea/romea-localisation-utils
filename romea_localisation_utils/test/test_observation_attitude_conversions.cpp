// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsAttitudeConversion : public ::testing::Test
{
public :

  TestObsAttitudeConversion():
    stamp(1000),
    frame_id("foo"),
    romea_obs_attitude(),
    romea_obs_attitude_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_obs_attitude.Y(romea::ObservationAttitude::ROLL)=1;
    romea_obs_attitude.Y(romea::ObservationAttitude::PITCH)=2;
    fillEigenCovariance(romea_obs_attitude.R());
    romea::to_ros_msg(stamp,frame_id,romea_obs_attitude,romea_obs_attitude_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationAttitude romea_obs_attitude;
  romea_localisation_msgs::msg::ObservationAttitudeStamped romea_obs_attitude_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsAttitudeConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(romea_obs_attitude_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_attitude_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(romea_obs_attitude_msg.observation_attitude.roll_angle,
                   romea_obs_attitude.Y(romea::ObservationAttitude::ROLL));
  EXPECT_DOUBLE_EQ(romea_obs_attitude_msg.observation_attitude.pitch_angle,
                   romea_obs_attitude.Y(romea::ObservationAttitude::PITCH));
  isSame(romea_obs_attitude_msg.observation_attitude.covariance,romea_obs_attitude.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsAttitudeConversion, fromRosMsgtoObs)
{
  romea::ObservationAttitude romea_obs_attitude_bis;
  romea::extractObs(romea_obs_attitude_msg,romea_obs_attitude_bis);
  EXPECT_DOUBLE_EQ(romea_obs_attitude_bis.Y(romea::ObservationAttitude::ROLL),
                   romea_obs_attitude.Y(romea::ObservationAttitude::ROLL));
  EXPECT_DOUBLE_EQ(romea_obs_attitude_bis.Y(romea::ObservationAttitude::PITCH),
                   romea_obs_attitude.Y(romea::ObservationAttitude::PITCH));
  isSame(romea_obs_attitude_bis.R(),romea_obs_attitude.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
