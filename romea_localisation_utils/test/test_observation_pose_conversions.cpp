// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsPoseConversion : public ::testing::Test
{
public :

  TestObsPoseConversion():
    stamp(1000),
    frame_id("foo"),
    romea_obs_pose(),
    romea_obs_pose_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_obs_pose.Y(romea::ObservationPose::POSITION_X)=1;
    romea_obs_pose.Y(romea::ObservationPose::POSITION_Y)=2;
    romea_obs_pose.Y(romea::ObservationPose::ORIENTATION_Z)=3;
    romea_obs_pose.levelArm.x()=4;
    romea_obs_pose.levelArm.y()=5;
    romea_obs_pose.levelArm.z()=6;
    fillEigenCovariance(romea_obs_pose.R());
    romea::to_ros_msg(stamp,frame_id,romea_obs_pose,romea_obs_pose_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationPose romea_obs_pose;
  romea_localisation_msgs::msg::ObservationPose2DStamped romea_obs_pose_msg;

};

//-----------------------------------------------------------------------------
TEST_F(TestObsPoseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(romea_obs_pose_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_pose_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.x,
                   romea_obs_pose.Y(romea::ObservationPose::POSITION_X));
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.y,
                   romea_obs_pose.Y(romea::ObservationPose::POSITION_Y));
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.yaw,
                   romea_obs_pose.Y(romea::ObservationPose::ORIENTATION_Z));
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.x,
                   romea_obs_pose.levelArm.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.y,
                   romea_obs_pose.levelArm.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.z,
                   romea_obs_pose.levelArm.z());
  isSame(romea_obs_pose_msg.observation_pose.pose.covariance,romea_obs_pose.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsPoseConversion, fromRosMsgtoObs)
{
  romea::ObservationPose romea_obs_pose_bis;
  romea::extract_obs(romea_obs_pose_msg,romea_obs_pose_bis);
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.Y(romea::ObservationPose::POSITION_X),
                   romea_obs_pose.Y(romea::ObservationPose::POSITION_X));
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.Y(romea::ObservationPose::POSITION_Y),
                   romea_obs_pose.Y(romea::ObservationPose::POSITION_Y));
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.Y(romea::ObservationPose::ORIENTATION_Z),
                   romea_obs_pose.Y(romea::ObservationPose::ORIENTATION_Z));
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.levelArm.x(),romea_obs_pose.levelArm.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.levelArm.y(),romea_obs_pose.levelArm.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_bis.levelArm.z(),romea_obs_pose.levelArm.z());
  isSame(romea_obs_pose_bis.R(),romea_obs_pose.R());
}


//-----------------------------------------------------------------------------
class TestPoseConversion : public ::testing::Test
{
public :

  TestPoseConversion():
    stamp(1000),
    frame_id("foo"),
    romea_pose(),
    romea_obs_pose_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_pose.position.x()=1;
    romea_pose.position.y()=2;
    romea_pose.yaw=3;
    fillEigenCovariance(romea_pose.covariance);
    romea::to_ros_msg(stamp,frame_id,romea_pose,romea_obs_pose_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::Pose2D romea_pose;
  romea_localisation_msgs::msg::ObservationPose2DStamped romea_obs_pose_msg;

};

//-----------------------------------------------------------------------------
TEST_F(TestPoseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(romea_obs_pose_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_pose_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.x,romea_pose.position.x());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.position.y,romea_pose.position.y());
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.pose.yaw,romea_pose.yaw);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.x,0);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.y,0);
  EXPECT_DOUBLE_EQ(romea_obs_pose_msg.observation_pose.level_arm.z,0);
  isSame(romea_obs_pose_msg.observation_pose.pose.covariance,romea_pose.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
