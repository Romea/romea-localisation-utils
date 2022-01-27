#ifndef __LocalisationUpdaterPluginBase_HPP__
#define __LocalisationUpdaterPluginBase_HPP__

//std
#include <mutex>

//ros
#include <ros/ros.h>

//romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>


namespace romea {


class LocalisationUpdaterPluginBase
{
public:

  LocalisationUpdaterPluginBase(){}

  virtual bool heartBeatCallback(const Duration & duration)=0;

  virtual DiagnosticReport getReport() = 0;
};

}

#endif
