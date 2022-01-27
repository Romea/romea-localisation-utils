#ifndef __LocalisationFilterBase_HPP__
#define __LocalisationFilterBase_HPP__

//ros
#include <ros/ros.h>

//romea
#include<romea_core_localisation/LocalisationFSMState.hpp>

namespace romea
{

class LocalisationApplicationBase
{
public :

  LocalisationApplicationBase(){}

   virtual ~LocalisationApplicationBase()=default;

   virtual void configure(ros::NodeHandle &nh, ros::NodeHandle &private_nh)=0;

   virtual void timerCallback(const ros::TimerEvent & event)=0;

   virtual LocalisationFSMState getFSMState()=0;

   virtual void reset()=0;

};

}

#endif
