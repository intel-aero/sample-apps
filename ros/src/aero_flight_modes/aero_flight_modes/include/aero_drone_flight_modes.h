
#pragma once

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

class AeroDrone
{
public:
  AeroDrone();
  ~AeroDrone();

  // Actions
  bool arm();
  bool takeoff();
  bool land();

private:
  void resetHome();
  void getHomeGeoPoint();
  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
  void watchFlightModeThread();
  void printFlightModeCB(const mavros_msgs::StateConstPtr& mode);

  ros::NodeHandle nh_;
  mavros_msgs::HomePosition home_{};
  bool home_set_ = false;
  ros::Rate rate_ = ros::Rate(20.0);
  boost::thread* thread_watch_flight_mode_ = nullptr;  // for watching Aero's flight modes
};
