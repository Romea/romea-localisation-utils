#ifndef __LocalisationFilterParams_HPP__
#define __LocalisationFilterParams_HPP__


//ros
#include <ros/ros.h>

//romea
#include "romea_core_localisation/LocalisationStoppingCriteria.hpp"
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

namespace romea {


LocalisationStoppingCriteria loadStoppingCriteria(ros::NodeHandle &filter_nh);

size_t loadNumberOfParticles(ros::NodeHandle & filter_nh);

size_t loadStatePoolSize(ros::NodeHandle & filter_nh);

LocalisationUpdaterTriggerMode loadTriggerMode(ros::NodeHandle & filter_nh, std::string updater_name);

std::string loadTopicName(ros::NodeHandle & filter_nh, std::string updater_name);

double loadMinimalRate(ros::NodeHandle & filter_nh,std::string updater_name);

double loadMaximalMahalanobisDistance(ros::NodeHandle & filter_nh, std::string updater_name);

std::string logFilename(ros::NodeHandle & filter_nh, std::string updater_name);

}

#endif
