// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_BASE_HPP_
#define ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_BASE_HPP_


// ros
#include <rclcpp/rclcpp.hpp>

// romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>


namespace romea
{


class LocalisationUpdaterInterfaceBase
{
public:
  LocalisationUpdaterInterfaceBase() {}

  virtual bool heartbeat_callback(const Duration & duration) = 0;

  virtual DiagnosticReport get_report() = 0;
};

}  // namespace romea

#endif  // ROMEA_LOCALISATION_UTILS__FILTER__LOCALISATION_UPDATER_INTERFACE_BASE_HPP_
