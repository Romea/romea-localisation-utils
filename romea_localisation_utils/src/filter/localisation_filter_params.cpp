#include "romea_localisation_utils/filter/localisation_filter_params.hpp"
#include <romea_common_utils/params/ros_param.hpp>

#include <ros/file_log.h>

namespace
{
const double DEFAULT_MAXIMAL_MAHALANOBIS_DISTANCE = std::numeric_limits<double>::max();
}

namespace romea
{

LocalisationStoppingCriteria loadStoppingCriteria(ros::NodeHandle &filter_nh)
{
  LocalisationStoppingCriteria stoppingCriteria;

  double maximal_dead_recknoning_travelled_distance;
  if(filter_nh.getParam("predictor/maximal_dead_recknoning_travelled_distance",maximal_dead_recknoning_travelled_distance))
  {
    stoppingCriteria.maximalTravelledDistanceInDeadReckoning = maximal_dead_recknoning_travelled_distance;
  }
  else
  {
    ROS_WARN_STREAM(" no maximal_dead_recknoning_travelled_distance has been specified, are you sure? ");
  }

  double maximal_dead_recknoning_elapsed_time;
  if(filter_nh.getParam("predictor/maximal_dead_recknoning_elapsed_time",maximal_dead_recknoning_elapsed_time))
  {
    stoppingCriteria.maximalDurationInDeadReckoning=durationFromSecond(maximal_dead_recknoning_elapsed_time);
  }
  else
  {
    ROS_WARN_STREAM(" no maximal_dead_recknoning_elapsed_time has been specified, are you sure? ");
  }

  double maximal_position_circular_error_probability;
  if(filter_nh.getParam("maximal_position_circular_error_probability",maximal_position_circular_error_probability))
  {
    stoppingCriteria.maximalPositionCircularErrorProbability = maximal_position_circular_error_probability;
  }
  else
  {
    ROS_WARN_STREAM(" no maximal_position_circular_error_probability has been specified, are you sure? ");
  }

  return stoppingCriteria;
}

//-----------------------------------------------------------------------------
size_t loadNumberOfParticles(ros::NodeHandle & filter_nh)
{
  return static_cast<size_t>(load_param<int>(filter_nh,"filter/number_of_particles"));
}

//-----------------------------------------------------------------------------
size_t loadStatePoolSize(ros::NodeHandle & filter_nh)
{
  return static_cast<size_t>(load_param<int>(filter_nh,"filter/state_pool_size"));
}

//-----------------------------------------------------------------------------
LocalisationUpdaterTriggerMode loadTriggerMode(ros::NodeHandle & filter_nh, std::string updater_name)
{
  return toTriggerMode(load_param<std::string>(filter_nh,updater_name+"/trigger"));
}

//-----------------------------------------------------------------------------
std::string loadTopicName(ros::NodeHandle & filter_nh, std::string updater_name)
{
  return load_param<std::string>(filter_nh,updater_name+"/topic");
}

//-----------------------------------------------------------------------------
double loadMinimalRate(ros::NodeHandle & filter_nh, std::string updater_name)
{
  return load_param<double>(filter_nh,updater_name+"/minimal_rate");
}

//-----------------------------------------------------------------------------
double loadMaximalMahalanobisDistance(ros::NodeHandle & filter_nh, std::string updater_name)
{

  double maximal_mahalanobis_distance;
  std::string maximal_mahalanobis_distance_name = updater_name+"/mahalanobis_distance_threshold";

  if(!filter_nh.param(maximal_mahalanobis_distance_name,
                      maximal_mahalanobis_distance,
                      DEFAULT_MAXIMAL_MAHALANOBIS_DISTANCE))
  {
    ROS_WARN_STREAM("No "<<maximal_mahalanobis_distance_name <<" has been specified, are you sure?");
  }

  return maximal_mahalanobis_distance;
}

//-----------------------------------------------------------------------------
std::string logFilename(ros::NodeHandle & filter_nh, std::string updater_name)
{
  std::string filename;
  if(filter_nh.param("debug",false))
  {
    filename = filter_nh.getNamespace()+"/"+updater_name+".dat";
    std::replace_copy(filename.begin()+1, filename.end(), filename.begin()+1, '/', '-');
    filename = ros::file_log::getLogDirectory()+filename;
  }
  return filename;
}


}
