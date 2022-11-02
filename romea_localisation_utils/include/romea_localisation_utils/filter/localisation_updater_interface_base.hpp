#ifndef __LocalisationUpdaterPluginBase_HPP__
#define __LocalisationUpdaterPluginBase_HPP__


//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>


namespace romea {


class LocalisationUpdaterInterfaceBase
{
public:

  LocalisationUpdaterInterfaceBase(){}

  virtual bool heartbeat_callback(const Duration & duration)=0;

  virtual DiagnosticReport get_report() = 0;
};

}

#endif
