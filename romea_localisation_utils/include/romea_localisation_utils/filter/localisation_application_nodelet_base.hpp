#ifndef __LocalisationFilterNodeletBase_HPP__
#define __LocalisationFilterNodeletBase_HPP__

#include <romea_lifecycle/LifecycleNodelet.hpp>
#include <romea_localisation_msgs/CheckLocalisationStatus.h>
#include "localisation_application_base.hpp"

namespace romea
{

class LocalisationApplicationNodeletBase : public LifecycleNodelet
{

public :

  LocalisationApplicationNodeletBase();

  virtual ~LocalisationApplicationNodeletBase()=default;

  virtual void onConfigure()override;

  virtual void onActivate()override;

  virtual void onDeactivate()override;

protected :

  virtual void configureLocalisation_()=0;

  void configureTimer_();

  void configureGetStatusService_();

  bool checkStatusCallback_(romea_localisation_msgs::CheckLocalisationStatus::Request & ,
                            romea_localisation_msgs::CheckLocalisationStatus::Response & response);

protected :

  ros::Timer timer_;
  ros::ServiceServer check_status_service_;
  std::unique_ptr<LocalisationApplicationBase> localisation_;

};

}


#endif
