// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// std
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "../test/test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsAngularSpeedConversion : public ::testing::Test
{
public:
  TestObsAngularSpeedConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_obs_angular_speed(),
    ros_obs_angular_speed_msg()
  {
  }

  void SetUp()override
  {
    romea_obs_angular_speed.Y() = 1;
    romea_obs_angular_speed.R() = 4;
    romea::to_ros_msg(stamp, frame_id, romea_obs_angular_speed, ros_obs_angular_speed_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationAngularSpeed romea_obs_angular_speed;
  romea_localisation_msgs::msg::ObservationAngularSpeedStamped ros_obs_angular_speed_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsAngularSpeedConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_obs_angular_speed_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_obs_angular_speed_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(
    ros_obs_angular_speed_msg.observation_angular_speed.velocity,
    romea_obs_angular_speed.Y());
  EXPECT_DOUBLE_EQ(
    ros_obs_angular_speed_msg.observation_angular_speed.std,
    std::sqrt(romea_obs_angular_speed.R()));
}

//-----------------------------------------------------------------------------
TEST_F(TestObsAngularSpeedConversion, fromRosMsgtoObs)
{
  romea::ObservationAngularSpeed romea_obs_angular_speed_bis;
  romea::extract_obs(ros_obs_angular_speed_msg, romea_obs_angular_speed_bis);

  EXPECT_DOUBLE_EQ(romea_obs_angular_speed_bis.Y(), romea_obs_angular_speed.Y());
  EXPECT_DOUBLE_EQ(romea_obs_angular_speed_bis.R(), romea_obs_angular_speed.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
