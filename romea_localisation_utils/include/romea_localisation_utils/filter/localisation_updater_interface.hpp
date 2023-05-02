// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_localisation_utils/filter/localisation_parameters.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface_base.hpp"
#include "romea_localisation_utils/conversions/observation_conversions.hpp"


namespace romea
{

template<typename Filter_, typename Updater_, typename Msg>
class LocalisationUpdaterInterface : public LocalisationUpdaterInterfaceBase
{
public:
  using Filter = Filter_;
  using Updater = Updater_;
  using Observation = typename Updater_::Observation;

public:
  LocalisationUpdaterInterface(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name);

  void process_message(typename Msg::ConstSharedPtr msg);

  void load_updater(std::unique_ptr<Updater> updater);

  void register_filter(std::shared_ptr<Filter> filter);

  bool heartbeat_callback(const Duration & duration) override;

  DiagnosticReport get_report() override;

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Updater> updater_;
  std::shared_ptr<rclcpp::Subscription<Msg>> sub_;
};

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
LocalisationUpdaterInterface<Filter_, Updater_, Msg>::LocalisationUpdaterInterface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name)
: LocalisationUpdaterInterfaceBase(),
  filter_(nullptr),
  updater_(nullptr),
  sub_()
{
  auto callback = std::bind(
    &LocalisationUpdaterInterface::process_message,
    this, std::placeholders::_1);

  sub_ = node->create_subscription<Msg>(topic_name, best_effort(1), callback);
}


//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::load_updater(
  std::unique_ptr<Updater> updater)
{
  updater_.swap(updater);
}

//-----------------------------------------------------------------------------
template<typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::register_filter(
  std::shared_ptr<Filter> filter)
{
  filter_ = filter;
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
void LocalisationUpdaterInterface<Filter_, Updater_, Msg>::process_message(
  typename Msg::ConstSharedPtr msg)
{
  Duration duration = extract_duration(*msg);

  Observation observation = extract_obs<Observation>(*msg);

  auto updateFunction = std::bind(
    &Updater::update,
    updater_.get(),
    std::placeholders::_1,
    std::move(observation),
    std::placeholders::_2,
    std::placeholders::_3);

  filter_->process(duration, std::move(updateFunction));
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
bool LocalisationUpdaterInterface<Filter_, Updater_, Msg>::heartbeat_callback(
  const Duration & duration)
{
  return updater_->heartBeatCallback(duration);
}

//-----------------------------------------------------------------------------
template<class Filter_, class Updater_, class Msg>
DiagnosticReport LocalisationUpdaterInterface<Filter_, Updater_, Msg>::get_report()
{
  return updater_->getReport();
}

//-----------------------------------------------------------------------------
template<typename UpdaterInterface>
std::unique_ptr<UpdaterInterface> make_updater_interface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  std::string topic_name = get_updater_topic_name(node, updater_name);
  return std::make_unique<UpdaterInterface>(node, topic_name);
}

//-----------------------------------------------------------------------------
template<typename UpdaterInterface>
std::unique_ptr<UpdaterInterface> make_updater_interface(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  std::shared_ptr<typename UpdaterInterface::Filter> filter,
  std::unique_ptr<typename UpdaterInterface::Updater> updater)
{
  auto interface = make_updater_interface<UpdaterInterface>(node, updater_name);
  interface->load_updater(std::move(updater));
  interface->register_filter(filter);
  return interface;
}

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_HPP_
