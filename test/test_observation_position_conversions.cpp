// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "../test/test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsPositionConversion : public ::testing::Test
{
public:
  TestObsPositionConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_obs_position(),
    romea_obs_position_msg()
  {
  }

  void SetUp()override
  {
    romea_obs_position.Y(romea::ObservationPosition::POSITION_X) = 1;
    romea_obs_position.Y(romea::ObservationPosition::POSITION_Y) = 2;
    romea_obs_position.levelArm.x() = 4;
    romea_obs_position.levelArm.y() = 5;
    romea_obs_position.levelArm.z() = 6;
    fillEigenCovariance(romea_obs_position.R());
    romea::to_ros_msg(stamp, frame_id, romea_obs_position, romea_obs_position_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationPosition romea_obs_position;
  romea_localisation_msgs::msg::ObservationPosition2DStamped romea_obs_position_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsPositionConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(romea_obs_position_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_position_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.x,
    romea_obs_position.Y(romea::ObservationPosition::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.y,
    romea_obs_position.Y(romea::ObservationPosition::POSITION_Y));

  isSame(romea_obs_position_msg.observation_position.position.covariance, romea_obs_position.R());
}

//-----------------------------------------------------------------------------
TEST_F(TestObsPositionConversion, fromRosMsgtoObs)
{
  romea::ObservationPosition romea_obs_position_bis;
  romea::extract_obs(romea_obs_position_msg, romea_obs_position_bis);
  EXPECT_DOUBLE_EQ(
    romea_obs_position_bis.Y(romea::ObservationPosition::POSITION_X),
    romea_obs_position.Y(romea::ObservationPosition::POSITION_X));
  EXPECT_DOUBLE_EQ(
    romea_obs_position_bis.Y(romea::ObservationPosition::POSITION_Y),
    romea_obs_position.Y(romea::ObservationPosition::POSITION_Y));
  isSame(romea_obs_position_bis.R(), romea_obs_position.R());
}


//-----------------------------------------------------------------------------
class TestPositionConversion : public ::testing::Test
{
public:
  TestPositionConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_position(),
    romea_obs_position_msg()
  {
  }

  void SetUp()override
  {
    romea_position.position.x() = 1;
    romea_position.position.y() = 2;
    fillEigenCovariance(romea_position.covariance);
    romea::to_ros_msg(stamp, frame_id, romea_position, romea_obs_position_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::Position2D romea_position;
  romea_localisation_msgs::msg::ObservationPosition2DStamped romea_obs_position_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPositionConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(romea_obs_position_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(romea_obs_position_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.x,
    romea_position.position.x());
  EXPECT_DOUBLE_EQ(
    romea_obs_position_msg.observation_position.position.y,
    romea_position.position.y());
  isSame(
    romea_obs_position_msg.observation_position.position.covariance,
    romea_position.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
