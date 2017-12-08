#ifndef __AERO_DRONE_H__
#define __AERO_DRONE_H__

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

class AeroDrone
{
public:
  bool arm();
  bool takeoff();
  bool land();
  AeroDrone();
  ~AeroDrone();

private:
  void resetHome();
  void getHomeGps();
  void setHomeGpsCB(const sensor_msgs::NavSatFixConstPtr& msg);

  ros::NodeHandle nh_;
  sensor_msgs::NavSatFix home_gps_{};
  bool is_home_gps_set_ = false;
  ros::Rate rate_ = ros::Rate(20.0);

  boost::thread* thread_watch_alt_ = nullptr;  // for watching drone's altitude
  void watchAltitudeThread();
  void getRelativeAltitudeCB(const std_msgs::Float64ConstPtr& msg, std_msgs::Float64* relative_altitude);
};

#endif
