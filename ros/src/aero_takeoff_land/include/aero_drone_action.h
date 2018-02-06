
#pragma once

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
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
  void watchAltitudeThread();
  void getRelativeAltitudeCB(const std_msgs::Float64ConstPtr& msg, std_msgs::Float64* relative_altitude);

  ros::NodeHandle nh_;
  mavros_msgs::HomePosition home_{};
  bool home_set_ = false;
  ros::Rate rate_ = ros::Rate(1.0);
  boost::thread* thread_watch_alt_ = nullptr;  // for watching drone's altitude
};
