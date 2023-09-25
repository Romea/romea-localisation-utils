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

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_FACTORY_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_FACTORY_HPP_

// std
#include <memory>
#include <string>
#include <utility>

#include "localisation_parameters.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_kalman_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name)
{
  return std::make_unique<Updater>(
    updater_name,
    get_updater_minimal_rate(node, updater_name),
    toTriggerMode(get_updater_trigger_mode(node, updater_name)),
    get_updater_mahalanobis_distance_rejection_threshold(node, updater_name),
    get_log_filename(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_particle_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name)
{
  return std::make_unique<Updater>(
    updater_name,
    get_updater_minimal_rate(node, updater_name),
    toTriggerMode(get_updater_trigger_mode(node, updater_name)),
    get_updater_mahalanobis_distance_rejection_threshold(node, updater_name),
    get_filter_number_of_particles(node),
    get_log_filename(node, updater_name));
}

//-----------------------------------------------------------------------------
template<class Updater, FilterType FilterType_>
std::unique_ptr<Updater> make_exteroceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name)
{
  if constexpr (FilterType_ == KALMAN) {
    return make_kalman_exteroceptive_updater<Updater>(node, updater_name);
  } else {
    return make_particle_exteroceptive_updater<Updater>(node, updater_name);
  }
}

//-----------------------------------------------------------------------------
template<class Updater>
std::unique_ptr<Updater> make_proprioceptive_updater(
  std::shared_ptr<rclcpp::Node> & node,
  const std::string & updater_name)
{
  return std::make_unique<Updater>(updater_name, get_updater_minimal_rate(node, updater_name));
}


//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_kalman_predictor(std::shared_ptr<rclcpp::Node> & node)
{
  return std::make_unique<Predictor>(
    durationFromSecond(get_predictor_maximal_dead_reckoning_elapsed_time(node)),
    get_predictor_maximal_dead_reckoning_travelled_distance(node),
    get_predictor_maximal_circular_error_probable(node));
}

//-----------------------------------------------------------------------------
template<class Predictor>
std::unique_ptr<Predictor> make_particle_predictor(std::shared_ptr<rclcpp::Node> & node)
{
  return std::make_unique<Predictor>(
    durationFromSecond(get_predictor_maximal_dead_reckoning_elapsed_time(node)),
    get_predictor_maximal_dead_reckoning_travelled_distance(node),
    get_predictor_maximal_circular_error_probable(node),
    get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class Predictor, FilterType FilterType_>
std::unique_ptr<Predictor> make_predictor(std::shared_ptr<rclcpp::Node> & node)
{
  if constexpr (FilterType_ == KALMAN) {
    return make_kalman_predictor<Predictor>(node);
  } else {
    return make_particle_predictor<Predictor>(node);
  }
}

//-----------------------------------------------------------------------------
template<class Filter>
std::unique_ptr<Filter> make_kalman_filter(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Filter>(get_filter_state_pool_size(node));
}

//-----------------------------------------------------------------------------
template<class Filter>
std::unique_ptr<Filter> make_particle_filter(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Filter>(
    get_filter_state_pool_size(node),
    get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class Filter, FilterType FilterType_>
std::unique_ptr<Filter> make_filter(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == KALMAN) {
    return make_kalman_filter<Filter>(node);
  } else {
    return make_particle_filter<Filter>(node);
  }
}

//-----------------------------------------------------------------------------
template<class Filter, class Predictor, FilterType FilterType_>
std::unique_ptr<Filter> make_filter(std::shared_ptr<rclcpp::Node> node)
{
  auto filter = make_filter<Filter, FilterType_>(node);
  auto predictor = make_predictor<Predictor, FilterType_>(node);
  filter->registerPredictor(std::move(predictor));
  return filter;
}


//-----------------------------------------------------------------------------
template<class Results>
std::unique_ptr<Results> make_kalman_results(std::shared_ptr<rclcpp::Node>/*node*/)
{
  return std::make_unique<Results>();
}

//-----------------------------------------------------------------------------
template<class Results>
std::unique_ptr<Results> make_particle_results(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<Results>(get_filter_number_of_particles(node));
}

//-----------------------------------------------------------------------------
template<class Results, FilterType FilterType_>
std::unique_ptr<Results> make_results(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == KALMAN) {
    return make_kalman_results<Results>(node);
  } else {
    return make_particle_results<Results>(node);
  }
}

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_FACTORY_HPP_
