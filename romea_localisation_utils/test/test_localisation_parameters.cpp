// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "test_helper.h"
#include "romea_localisation_utils/filter/localisation_parameters.hpp"


//-----------------------------------------------------------------------------
class TestLocalisationFilterParams : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions no;
    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_localisation_filter_parameters.yaml"});
    node = std::make_shared<rclcpp::Node>("test_localisation_filter_parameters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalDeadReckoningTravelledDistance)
{
  romea::declare_predictor_maximal_dead_reckoning_travelled_distance(node);
  EXPECT_DOUBLE_EQ(romea::get_predictor_maximal_dead_reckoning_travelled_distance(node),10.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalDeadReckoningElapsedTime)
{
  romea::declare_predictor_maximal_dead_reckoning_elapsed_time(node);
  EXPECT_DOUBLE_EQ(romea::get_predictor_maximal_dead_reckoning_elapsed_time(node),3.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetPredictorMaximalCircularErrorProbable)
{
  romea::declare_predictor_maximal_circular_error_probable(node);
  EXPECT_DOUBLE_EQ(romea::get_predictor_maximal_circular_error_probable(node),0.2);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetFilterNumberOfParticles)
{
  romea::declare_filter_number_of_particles(node);
  EXPECT_EQ(romea::get_filter_number_of_particles(node),200);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetFilterStatePoolSize)
{
  romea::declare_filter_state_pool_size(node);
  EXPECT_EQ(romea::get_filter_state_pool_size(node),1000);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterTriggerMode)
{
  romea::declare_updater_trigger_mode(node,"position_updater");
  EXPECT_EQ(romea::get_updater_trigger_mode(node,"position_updater"),"always");
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterTopicName)
{
  romea::declare_updater_topic_name(node,"pose_updater");
  EXPECT_EQ(romea::get_updater_topic_name(node,"pose_updater"),"pose");
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyTopicName)
{
  romea::declare_updater_topic_name(node,"foo");
  EXPECT_EQ(romea::get_updater_topic_name(node,"foo"),"");
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterMinimalRate)
{
  romea::declare_updater_minimal_rate(node,"angular_speed_updater");
  EXPECT_EQ(romea::get_updater_minimal_rate(node,"angular_speed_updater"),10);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyMinimalRate)
{
  romea::declare_updater_minimal_rate(node,"bar");
  EXPECT_EQ(romea::get_updater_minimal_rate(node,"bar"),0);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterMahalanobisDistance)
{
  romea::declare_updater_mahalanobis_distance_rejection_threshold(node,"position_updater");
  EXPECT_DOUBLE_EQ(romea::get_updater_mahalanobis_distance_rejection_threshold(node,"position_updater"),3);
}

//-----------------------------------------------------------------------------
TEST_F(TestLocalisationFilterParams, checkGetUpdaterEmptyMahalanobisDistance)
{
  romea::declare_updater_mahalanobis_distance_rejection_threshold(node,"bar");
  EXPECT_DOUBLE_EQ(romea::get_updater_mahalanobis_distance_rejection_threshold(node,"bar"),std::numeric_limits<double>::max());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
