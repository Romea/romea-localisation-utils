// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <limits>
#include <memory>
#include <string>

// romea
#include "romea_localisation_utils/filter/localisation_parameters.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

// #include <ros/file_log.h>

namespace
{

const double DEFAULT_MAXIMAL_MAHALANOBIS_DISTANCE = std::numeric_limits<double>::max();
const double DEFAULT_MAXIMAL_POSITION_CIRCULAR_ERROR_PROBABLE = 100;
const double DEFAULT_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE = 100000;
const double DEFAULT_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME = 3600;

const char PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME[] =
  "predictor.maximal_dead_recknoning_travelled_distance";
const char PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME[] =
  "predictor.maximal_dead_recknoning_elapsed_time";
const char PREDICTOR_MAXIMAL_POSITION_CIRCULAR_ERROR_PROBABLE_PARAM_NAME[] =
  "predictor.maximal_position_circular_error_probability";

const char FILTER_NUMBER_OF_PARTICLES_PARAM_NAME[] =
  "filter.number_of_particles";
const char FILTER_STATE_POOL_SIZE_PARAM_NAME[] =
  "filter.state_pool_size";

const char UPDATER_TRIGGER_PARAM_NAME[] =
  "trigger";
const char UPDATER_TOPIC_PARAM_NAME[] =
  "topic";
const char UPDATER_MINIMAL_RATE_PARAM_NAME[] =
  "minimal_rate";
const char UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME[] =
  "mahalanobis_distance_rejection_threshold";

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
void declare_predictor_parameters(std::shared_ptr<rclcpp::Node> node)
{
  declare_predictor_maximal_dead_reckoning_travelled_distance(node);
  declare_predictor_maximal_dead_reckoning_elapsed_time(node);
  declare_predictor_maximal_circular_error_probable(node);
}

//-----------------------------------------------------------------------------
void declare_predictor_maximal_dead_reckoning_travelled_distance(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME,
    DEFAULT_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE);
}

//-----------------------------------------------------------------------------
double get_predictor_maximal_dead_reckoning_travelled_distance(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_TRAVELLED_DISTANCE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_predictor_maximal_dead_reckoning_elapsed_time(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME,
    DEFAULT_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME);
}

//-----------------------------------------------------------------------------
double get_predictor_maximal_dead_reckoning_elapsed_time(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(
    node, PREDICTOR_MAXIMAL_DEAD_RECKONING_ELAPSED_TIME_PARAM_NAME);
}
//-----------------------------------------------------------------------------
void declare_predictor_maximal_circular_error_probable(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<double>(
    node, PREDICTOR_MAXIMAL_POSITION_CIRCULAR_ERROR_PROBABLE_PARAM_NAME,
    DEFAULT_MAXIMAL_POSITION_CIRCULAR_ERROR_PROBABLE);
}

//-----------------------------------------------------------------------------
double get_predictor_maximal_circular_error_probable(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(
    node, PREDICTOR_MAXIMAL_POSITION_CIRCULAR_ERROR_PROBABLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_kalman_filter_parameters(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_state_pool_size(node);
}

//-----------------------------------------------------------------------------
void declare_particle_filter_parameters(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_state_pool_size(node);
  declare_filter_number_of_particles(node);
}

//-----------------------------------------------------------------------------
void declare_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FILTER_NUMBER_OF_PARTICLES_PARAM_NAME);
}

//-----------------------------------------------------------------------------
size_t get_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node)
{
  return static_cast<size_t>(get_parameter<int>(node, FILTER_NUMBER_OF_PARTICLES_PARAM_NAME));
}

//-----------------------------------------------------------------------------
void declare_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FILTER_STATE_POOL_SIZE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
size_t get_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node)
{
  return static_cast<size_t>(get_parameter<int>(node, FILTER_STATE_POOL_SIZE_PARAM_NAME));
}

//-----------------------------------------------------------------------------
void declare_proprioceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_updater_topic_name(node, updater_name);
  declare_updater_minimal_rate(node, updater_name);
}

//-----------------------------------------------------------------------------
void declare_exteroceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_updater_topic_name(node, updater_name);
  declare_updater_minimal_rate(node, updater_name);
  declare_updater_trigger_mode(node, updater_name);
  declare_updater_mahalanobis_distance_rejection_threshold(node, updater_name);
}


//-----------------------------------------------------------------------------
void declare_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_parameter_with_default<std::string>(
    node, updater_name, UPDATER_TRIGGER_PARAM_NAME, "");
}

//-----------------------------------------------------------------------------
std::string get_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  return get_parameter<std::string>(
    node, updater_name, UPDATER_TRIGGER_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_updater_topic_name(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_parameter_with_default<std::string>(
    node, updater_name, UPDATER_TOPIC_PARAM_NAME, "");
}

//-----------------------------------------------------------------------------
std::string get_updater_topic_name(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  return get_parameter<std::string>(
    node, updater_name, UPDATER_TOPIC_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name)
{
  declare_parameter_with_default<int>(
    node, updater_name, UPDATER_MINIMAL_RATE_PARAM_NAME, 0);
}

//-----------------------------------------------------------------------------
unsigned int get_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name)
{
  int minimal_rate = get_parameter<int>(
    node, updater_name, UPDATER_MINIMAL_RATE_PARAM_NAME);

  if (minimal_rate < 0) {
    throw(std::runtime_error("Invalid minimal rate for updater " + updater_name));
  }

  return static_cast<unsigned int>(minimal_rate);
}


//-----------------------------------------------------------------------------
void declare_updater_maximal_mahalanobis_distance(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name)
{
  declare_parameter_with_default<double>(
    node, updater_name,
    UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME, 0);
}

//-----------------------------------------------------------------------------
void declare_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name)
{
  declare_parameter_with_default<double>(
    node, updater_name,
    UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME,
    DEFAULT_MAXIMAL_MAHALANOBIS_DISTANCE);
}

//-----------------------------------------------------------------------------
double get_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name)
{
  return get_parameter<double>(
    node, updater_name,
    UPDATER_MAHALANOBIS_DISTANCE_REJECTION_THRESHOLD_PARAM_NAME);
}

}  // namespace romea
