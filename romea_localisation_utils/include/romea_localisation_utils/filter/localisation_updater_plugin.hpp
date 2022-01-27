#ifndef __LocalisationUpdaterPlugin_HPP__
#define __LocalisationUpdaterPlugin_HPP__

#include "localisation_filter_params.hpp"
#include "localisation_updater_plugin_base.hpp"
#include "../conversions/observation_conversions.hpp"
//#include <romea_core_localisation/LocalisationUpdater.hpp>

namespace romea {

template < typename Filter_, typename Updater_, typename Msg>
class LocalisationUpdaterPlugin : public LocalisationUpdaterPluginBase
{
public :

  using Filter = Filter_;
  using Updater = Updater_;
  using Observation = typename Updater_::Observation;

public :

  LocalisationUpdaterPlugin(ros::NodeHandle & nh,const std::string & topic_name);

  void processMesssage(const typename Msg::ConstPtr & msg);

  void loadUpdater(std::unique_ptr<Updater> updater);

  void registerFilter(std::shared_ptr<Filter> filter);

  virtual bool heartBeatCallback(const Duration & duration) override;

  virtual DiagnosticReport getReport() override;

private:

  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Updater> updater_;
  ros::Subscriber sub_;


};

//-----------------------------------------------------------------------------
template <typename Filter_, typename Updater_, typename Msg>
LocalisationUpdaterPlugin<Filter_,Updater_,Msg>::
LocalisationUpdaterPlugin(ros::NodeHandle & nh,const std::string & topic_name):
  LocalisationUpdaterPluginBase(),
  filter_(nullptr),
  updater_(nullptr),
  sub_()
{
  sub_= nh.subscribe<Msg>(topic_name, 1,&LocalisationUpdaterPlugin::processMesssage,this);
}


//-----------------------------------------------------------------------------
template <typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterPlugin<Filter_,Updater_,Msg>::loadUpdater(std::unique_ptr<Updater> updater)
{
  updater_.swap(updater);
}

//-----------------------------------------------------------------------------
template <typename Filter_, typename Updater_, typename Msg>
void LocalisationUpdaterPlugin<Filter_,Updater_,Msg>::registerFilter(std::shared_ptr<Filter> filter)
{
  filter_ = filter;
}

//-----------------------------------------------------------------------------
template <class Filter_, class Updater_, class Msg>
void LocalisationUpdaterPlugin <Filter_,Updater_,Msg>::processMesssage(const typename Msg::ConstPtr & msg)
{

  Duration duration=extractDuration(*msg);

  Observation observation = extractObs<Observation>(*msg);

  auto updateFunction = std::bind(&Updater::update,
                                  updater_.get(),
                                  std::placeholders::_1,
                                  std::move(observation),
                                  std::placeholders::_2,
                                  std::placeholders::_3);

  filter_->process(duration,std::move(updateFunction));
}

//-----------------------------------------------------------------------------
template <class Filter_, class Updater_, class Msg>
bool LocalisationUpdaterPlugin <Filter_,Updater_,Msg>::heartBeatCallback(const Duration & duration)
{
   return updater_->heartBeatCallback(duration);
}

//-----------------------------------------------------------------------------
template <class Filter_, class Updater_, class Msg>
DiagnosticReport LocalisationUpdaterPlugin <Filter_,Updater_,Msg>::getReport()
{
  return updater_->getReport();
}

//-----------------------------------------------------------------------------
template<typename UpdaterPlugin>
std::unique_ptr<UpdaterPlugin> makeUpdaterPlugin(ros::NodeHandle & nh,
                                                 ros::NodeHandle & filter_nh,
                                                 const std::string & updater_name)
{
  std::string topic_name = loadTopicName(filter_nh,updater_name);
  return std::make_unique<UpdaterPlugin>(nh,topic_name);
}

//-----------------------------------------------------------------------------
template<typename UpdaterPlugin>
std::unique_ptr<UpdaterPlugin> makeUpdaterPlugin(ros::NodeHandle & nh,
                                                 ros::NodeHandle & filter_nh,
                                                 const std::string & updater_name,
                                                 std::shared_ptr<typename UpdaterPlugin::Filter> filter,
                                                 std::unique_ptr<typename UpdaterPlugin::Updater> updater)
{
  auto plugin = makeUpdaterPlugin<UpdaterPlugin>(nh,filter_nh,updater_name);
  plugin->loadUpdater(std::move(updater));
  plugin->registerFilter(filter);
  return plugin;
}


}

#endif
