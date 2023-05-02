// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_PARAMETERS_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_PARAMETERS_HPP_

// std
#include <string>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_core_filtering/FilterType.hpp"


namespace romea
{


void declare_predictor_parameters(std::shared_ptr<rclcpp::Node> node);

void declare_predictor_maximal_dead_reckoning_travelled_distance(
  std::shared_ptr<rclcpp::Node> node);

double get_predictor_maximal_dead_reckoning_travelled_distance(std::shared_ptr<rclcpp::Node> node);

void declare_predictor_maximal_dead_reckoning_elapsed_time(std::shared_ptr<rclcpp::Node> node);

double get_predictor_maximal_dead_reckoning_elapsed_time(std::shared_ptr<rclcpp::Node> node);

void declare_predictor_maximal_circular_error_probable(std::shared_ptr<rclcpp::Node> node);

double get_predictor_maximal_circular_error_probable(std::shared_ptr<rclcpp::Node> node);


void declare_kalman_filter_parameters(std::shared_ptr<rclcpp::Node> node);

void declare_particle_filter_parameters(std::shared_ptr<rclcpp::Node> node);

template<FilterType FilterType_>
void declare_filter_parameters(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == KALMAN)
  {
    return declare_kalman_filter_parameters(node);
  } else {
    return declare_particle_filter_parameters(node);
  }
}

void declare_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node);

size_t get_filter_number_of_particles(std::shared_ptr<rclcpp::Node> node);

void declare_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node);

size_t get_filter_state_pool_size(std::shared_ptr<rclcpp::Node> node);


void declare_proprioceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

void declare_exteroceptive_updater_parameters(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

void declare_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

std::string get_updater_trigger_mode(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

void declare_updater_topic_name(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

std::string get_updater_topic_name(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name);

void declare_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name);

unsigned int get_updater_minimal_rate(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name);

void declare_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name);

double get_updater_mahalanobis_distance_rejection_threshold(
  std::shared_ptr<rclcpp::Node> node,
  std::string updater_name);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_PARAMETERS_HPP_
