// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation_utils/conversions/observation_course_conversions.hpp"


//-----------------------------------------------------------------------------
class TestObsCourseConversion : public ::testing::Test
{
public :

  TestObsCourseConversion():
    stamp(1000),
    frame_id("foo"),
    romea_obs_course(),
    ros_obs_course_msg()
  {
  }

  virtual void SetUp()override
  {
    romea_obs_course.Y()=1;
    romea_obs_course.R()=4;
    romea::to_ros_msg(stamp,frame_id,romea_obs_course,ros_obs_course_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::ObservationCourse romea_obs_course;
  romea_localisation_msgs::msg::ObservationCourseStamped ros_obs_course_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestObsCourseConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_obs_course_msg).nanoseconds(),stamp.nanoseconds());
  EXPECT_STREQ(ros_obs_course_msg.header.frame_id.c_str(),frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_obs_course_msg.observation_course.angle,romea_obs_course.Y());
  EXPECT_DOUBLE_EQ(ros_obs_course_msg.observation_course.std,std::sqrt(romea_obs_course.R()));
}

//-----------------------------------------------------------------------------
TEST_F(TestObsCourseConversion, fromRosMsgtoObs)
{
  romea::ObservationCourse romea_obs_course_bis;
  romea::extractObs(ros_obs_course_msg,romea_obs_course_bis);

  EXPECT_DOUBLE_EQ(romea_obs_course_bis.Y(),romea_obs_course.Y());
  EXPECT_DOUBLE_EQ(romea_obs_course_bis.R(),romea_obs_course.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
