// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsRangeConversion : public ::testing::Test
{
public :

  TestObsRangeConversion():
    stamp(1000),
    frame_id("foo"),
    romea_obs_range(),
    ros_obs_range_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_obs_range.Y()=1;
    romea_obs_range.R()=4;
    romea_obs_range.initiatorPosition.x()=1;
    romea_obs_range.initiatorPosition.y()=2;
    romea_obs_range.initiatorPosition.z()=3;
    romea_obs_range.responderPosition.x()=4;
    romea_obs_range.responderPosition.y()=5;
    romea_obs_range.responderPosition.z()=6;
    romea::to_ros_msg(stamp,frame_id,romea_obs_range,ros_obs_range_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationRange romea_obs_range;
  romea_localisation_msgs::msg::ObservationRangeStamped ros_obs_range_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsRangeConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_obs_range_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(ros_obs_range_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.range,romea_obs_range.Y());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.range_std,std::sqrt(romea_obs_range.R()));
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.initiator_antenna_position.x,romea_obs_range.initiatorPosition.x());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.initiator_antenna_position.y,romea_obs_range.initiatorPosition.y());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.initiator_antenna_position.z,romea_obs_range.initiatorPosition.z());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.responder_antenna_position.x,romea_obs_range.responderPosition.x());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.responder_antenna_position.y,romea_obs_range.responderPosition.y());
  EXPECT_DOUBLE_EQ(ros_obs_range_msg.observation_range.responder_antenna_position.z,romea_obs_range.responderPosition.z());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsRangeConversion, fromRosMsgtoObs)
{
  romea::ObservationRange romea_obs_range_bis;
  romea::extract_obs(ros_obs_range_msg,romea_obs_range_bis);
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.Y(),romea_obs_range.Y());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.R(),romea_obs_range.R());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.initiatorPosition.x(),romea_obs_range.initiatorPosition.x());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.initiatorPosition.y(),romea_obs_range.initiatorPosition.y());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.initiatorPosition.z(),romea_obs_range.initiatorPosition.z());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.responderPosition.x(),romea_obs_range.responderPosition.x());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.responderPosition.y(),romea_obs_range.responderPosition.y());
  EXPECT_DOUBLE_EQ(romea_obs_range_bis.responderPosition.z(),romea_obs_range.responderPosition.z());

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
