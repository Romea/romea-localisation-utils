#include "romea_localisation_utils/filter/localisation_application_nodelet_base.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
LocalisationApplicationNodeletBase::LocalisationApplicationNodeletBase():
  timer_(),
  check_status_service_(),
  localisation_()
{

}

//-----------------------------------------------------------------------------
void LocalisationApplicationNodeletBase::onConfigure()
{
  configureLocalisation_();
  configureGetStatusService_();
  configureTimer_();

}

//-----------------------------------------------------------------------------
void LocalisationApplicationNodeletBase::onActivate()
{
  timer_.start();
}

//-----------------------------------------------------------------------------
void LocalisationApplicationNodeletBase::onDeactivate()
{
  timer_.stop();
}

//-----------------------------------------------------------------------------
void LocalisationApplicationNodeletBase::configureTimer_()
{

  auto nh = getNodeHandle();

  timer_= nh.createTimer(ros::Rate(10),
                         &LocalisationApplicationBase::timerCallback,
                         localisation_.get(),
                         false,
                         false);

}

//-----------------------------------------------------------------------------
void LocalisationApplicationNodeletBase::configureGetStatusService_()
{
  auto nh = getNodeHandle();
  check_status_service_ =
      nh.advertiseService("check_location_status",
                          &LocalisationApplicationNodeletBase::checkStatusCallback_,
                          this);
}

//-----------------------------------------------------------------------------
bool LocalisationApplicationNodeletBase::checkStatusCallback_(
    romea_localisation_msgs::CheckLocalisationStatusRequest & request,
    romea_localisation_msgs::CheckLocalisationStatusResponse &)
{
  return static_cast<LocalisationFSMState>(request.expected_status)==localisation_->getFSMState();
}


}

