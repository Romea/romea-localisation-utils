#ifndef __LocalisationFilterFactory_HPP__
#define __LocalisationFilterFactory_HPP__

#include "localisation_filter_params.hpp"
#include <romea_core_filtering/FilterType.hpp>

namespace romea {

//-----------------------------------------------------------------------------
template <class Updater>
std::unique_ptr<Updater> makeKalmanExteroceptiveUpdater(ros::NodeHandle & nh,
                                                        const std::string & updater_name)
{
  return std::make_unique<Updater>(updater_name,
                                   loadMinimalRate(nh,updater_name),
                                   loadTriggerMode(nh,updater_name),
                                   loadMaximalMahalanobisDistance(nh,updater_name),
                                   logFilename(nh,updater_name));





}

//-----------------------------------------------------------------------------
template <class Updater>
std::unique_ptr<Updater> makeParticleExteroceptiveUpdater(ros::NodeHandle & nh,
                                                    const std::string & updater_name)
{
  return std::make_unique<Updater>(updater_name,
                                   loadMinimalRate(nh,updater_name),
                                   loadTriggerMode(nh,updater_name),
                                   loadNumberOfParticles(nh),
                                   loadMaximalMahalanobisDistance(nh,updater_name),
                                   logFilename(nh,updater_name));


}

//-----------------------------------------------------------------------------
template <class Updater , FilterType FilterType_>
std::unique_ptr<Updater> makeExteroceptiveUpdater(ros::NodeHandle & nh,
                                                  const std::string & updater_name)
{
  if constexpr(FilterType_ == KALMAN)
  {
    return makeKalmanExteroceptiveUpdater<Updater>(nh,updater_name);
  }
  else
  {
    return makeParticleExteroceptiveUpdater<Updater>(nh,updater_name);
  }
}

//-----------------------------------------------------------------------------
template <class Updater>
std::unique_ptr<Updater> makeProprioceptiveUpdater(ros::NodeHandle & nh,
                                                   const std::string & updater_name)
{
  return std::make_unique<Updater>(updater_name,loadMinimalRate(nh,updater_name));
}


//-----------------------------------------------------------------------------
template <class Predictor>
std::unique_ptr<Predictor> makeKalmanPredictor(ros::NodeHandle & nh)
{
  return std::make_unique<Predictor>(loadStoppingCriteria(nh));
}

//-----------------------------------------------------------------------------
template <class Predictor>
std::unique_ptr<Predictor> makeParticlePredictor(ros::NodeHandle & nh)
{
  return std::make_unique<Predictor>(loadStoppingCriteria(nh),
                                     loadNumberOfParticles(nh));
}

//-----------------------------------------------------------------------------
template <class Predictor, FilterType FilterType_>
std::unique_ptr<Predictor> makePredictor(ros::NodeHandle & nh)
{
  if constexpr(FilterType_ == KALMAN)
  {
    return makeKalmanPredictor<Predictor>(nh);
  }
  else
  {
    return makeParticlePredictor<Predictor>(nh);
  }
}

//-----------------------------------------------------------------------------
template <class Filter>
std::unique_ptr<Filter> makeKalmanFilter(ros::NodeHandle & nh)
{
  return std::make_unique<Filter>(loadStatePoolSize(nh));
}

//-----------------------------------------------------------------------------
template <class Filter>
std::unique_ptr<Filter> makeParticleFilter(ros::NodeHandle & nh)
{
  return std::make_unique<Filter>(loadStatePoolSize(nh),
                                  loadNumberOfParticles(nh));
}

//-----------------------------------------------------------------------------
template <class Filter, FilterType FilterType_>
std::unique_ptr<Filter> makeFilter(ros::NodeHandle & nh)
{
  if constexpr(FilterType_ == KALMAN)
  {
    return makeKalmanFilter<Filter>(nh);
  }
  else
  {
    return makeParticleFilter<Filter>(nh);
  }
}

//-----------------------------------------------------------------------------
template <class Filter, class Predictor, FilterType FilterType_>
std::unique_ptr<Filter> makeFilter(ros::NodeHandle & nh)
{
  auto filter = makeFilter<Filter,FilterType_>(nh);
  auto predictor = makePredictor<Predictor,FilterType_>(nh);
  filter->registerPredictor(std::move(predictor));
  return filter;
}


//-----------------------------------------------------------------------------
template <class Results>
std::unique_ptr<Results> makeKalmanResults(ros::NodeHandle &/*nh*/)
{
  return std::make_unique<Results>();
}

//-----------------------------------------------------------------------------
template <class Results>
std::unique_ptr<Results> makeParticleResults(ros::NodeHandle & nh)
{
  return std::make_unique<Results>(loadNumberOfParticles(nh));
}

//-----------------------------------------------------------------------------
template <class Results,FilterType FilterType_>
std::unique_ptr<Results> makeResults(ros::NodeHandle & nh)
{
  if constexpr(FilterType_ == KALMAN)
  {
    return makeKalmanResults<Results>(nh);
  }
  else
  {
    return makeParticleResults<Results>(nh);
  }
}

}

#endif
