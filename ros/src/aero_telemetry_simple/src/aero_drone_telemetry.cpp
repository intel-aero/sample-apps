
#include "aero_drone_telemetry.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatStatus.h>
#include <ros/ros.h>
#include <cmath>

void setHomePositionCB(const mavros_msgs::HomePositionConstPtr& home);
void setGPSFixCB(const sensor_msgs::NavSatFixConstPtr& gps_fix);
void setGPSRelativeAltCB(const std_msgs::Float64ConstPtr& rel_alt);
void setLandedStateCB(const mavros_msgs::ExtendedStateConstPtr& landed_state);
void setBatteryStateCB(const sensor_msgs::BatteryStateConstPtr& battery_state);

void AeroDrone::fetchTelemetryThread()
{
  // Home
  auto home_pos_sub = nh_.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
                                                               boost::bind(&AeroDrone::setHomePositionCB, this, _1));
  // GPS Fix
  auto gps_fix_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1,
                                                           boost::bind(&AeroDrone::setGPSFixCB, this, _1));
  // Relative altitude
  auto rel_alt_sub =
      nh_.subscribe<mavros_msgs::Altitude>("mavros/altitude", 1, boost::bind(&AeroDrone::setAltitudeCB, this, _1));
  // Landed state
  auto extended_state_sub = nh_.subscribe<mavros_msgs::ExtendedState>(
      "mavros/extended_state", 1, boost::bind(&AeroDrone::setExtendedStateCB, this, _1));

  // Battery state
  auto battery_state_sub = nh_.subscribe<sensor_msgs::BatteryState>(
      "mavros/battery", 1, boost::bind(&AeroDrone::setBatteryStateCB, this, _1));

  while (ros::ok())
  {
    ros::spinOnce();
    rate_.sleep();
  }
}

AeroDrone::AeroDrone()
{
  thread_telemetry_ = new boost::thread(boost::bind(&AeroDrone::fetchTelemetryThread, this));
}

AeroDrone::~AeroDrone()
{
  delete thread_telemetry_;
}

float AeroDrone::absoluteAltitude()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return altitude.amsl;
}

float AeroDrone::relativeAltitude()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return altitude.relative;
}

const mavros_msgs::HomePosition& AeroDrone::homePosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return home_pos_;
}

const sensor_msgs::NavSatFix& AeroDrone::gpsFix()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gps_fix_;
}

std::string AeroDrone::landedState()
{
  std::lock_guard<std::mutex> lock(mutex_);
  switch (extended_state_.landed_state)
  {
    case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
      return "In-air";
    case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
      return "Landing";
    case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
      return "On-ground";
    case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
      return "Takeoff";
    default:
      return "Undefined";
  }
}

const sensor_msgs::BatteryState& AeroDrone::batteryState()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return battery_state_;
}

// Printing global position parameters
void AeroDrone::setHomePositionCB(const mavros_msgs::HomePositionConstPtr& home)
{
  std::lock_guard<std::mutex> lock(mutex_);
  home_pos_ = *home;
}

void AeroDrone::setAltitudeCB(const mavros_msgs::AltitudeConstPtr& alt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  altitude = *alt;
}

void AeroDrone::setGPSFixCB(const sensor_msgs::NavSatFixConstPtr& gps_fix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gps_fix_ = *gps_fix;
}

void AeroDrone::setExtendedStateCB(const mavros_msgs::ExtendedStateConstPtr& landed_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  extended_state_ = *landed_state;
}

void AeroDrone::setBatteryStateCB(const sensor_msgs::BatteryStateConstPtr& batter_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  battery_state_ = *batter_state;
}
