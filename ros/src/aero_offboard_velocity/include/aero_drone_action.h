
#pragma once
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Altitude.h>
class AeroDrone
{
public:
  AeroDrone();
  ~AeroDrone();

  // Actions
  bool arm();
  bool takeoff();
  bool land();
  bool setOffboardMode();

  // velocity
  void setOffboardVelocityNed(float vx, float vy, float vz, float yaw);
  void setOffboardVelocityNed(float vx, float vy, float vz, float yaw, int count);
  void setOffboardVelocityBody(float vx, float vy, float vz, float yaw_rate);
  void setOffboardVelocityBody(float vx, float vy, float vz, float yaw_rate, int count);

private:
  void getHomeGeoPoint();
  void getAltitude();
  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
  void getAltitudeCB(const mavros_msgs::AltitudeConstPtr& altitude);
  float toRadFromDeg(float deg);
  ros::NodeHandle nh_;
  mavros_msgs::HomePosition home_{};
  mavros_msgs::Altitude altitude_{};
  ros::Publisher set_vel_pub_;
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient land_client_;
  bool home_set_ = false;
  bool altitude_received_ = false;
  int altitude_in_amsl_;
  ros::Rate rate_ = ros::Rate(20.0);
};
