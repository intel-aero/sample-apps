
#include "aero_drone_action.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

AeroDrone::AeroDrone()
{
  resetHome();
  getHomeGps();
}

AeroDrone::~AeroDrone()
{
  delete thread_watch_alt_;
}

bool AeroDrone::arm()
{
  if (!is_home_gps_set_)
  {
    ROS_ERROR("Can't arm: No GPS Fix!");
    return false;
  }

  auto arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if (arming_client.call(srv_arm) && srv_arm.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::takeoff()
{
  if (!is_home_gps_set_)
  {
    ROS_ERROR("Can't takeoff: No GPS Fix!");
    return false;
  }

  auto takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff{};

  srv_takeoff.request.altitude = home_gps_.altitude;
  srv_takeoff.request.latitude = home_gps_.latitude;
  srv_takeoff.request.longitude = home_gps_.longitude;

  // Thread that watch for change in altitude of Aero.
  thread_watch_alt_ = new boost::thread(boost::bind(&AeroDrone::watchAltitudeThread, this));

  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::land()
{
  if (!is_home_gps_set_)
  {
    ROS_ERROR("Can't land: No GPS Fix!");
    return false;
  }

  auto land_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};

  if (land_client.call(srv_land) && srv_land.response.success)
    return true;
  else
    return false;
}

void AeroDrone::resetHome()
{
  home_gps_.latitude = home_gps_.longitude = home_gps_.altitude = NAN;
}

void AeroDrone::getHomeGps()
{
  // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
  auto home_gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1,
                                                            boost::bind(&AeroDrone::setHomeGpsCB, this, _1));

  ROS_INFO("Waiting for Aero FC Home to be set...");
  while (ros::ok() && !is_home_gps_set_)
  {
    ros::spinOnce();
    rate_.sleep();
  }
}

// Callback that gets called periodically from MAVROS notifying Global Poistion of Aero FCU
void AeroDrone::setHomeGpsCB(const sensor_msgs::NavSatFixConstPtr& msg)
{
  home_gps_ = *msg;
  is_home_gps_set_ = true;
  ROS_INFO("Received Home: %lf, %lf, %lf", home_gps_.latitude, home_gps_.longitude, home_gps_.altitude);
}

// printing altitude
void AeroDrone::getRelativeAltitudeCB(const std_msgs::Float64ConstPtr& msg, std_msgs::Float64* relative_altitude)
{
  *relative_altitude = *msg;
  ROS_INFO("Altitude: [%gm]", relative_altitude->data);
}

// subscribing to mavros/global_position/rel_alt to get altitude updates
void AeroDrone::watchAltitudeThread()
{
  std_msgs::Float64 relative_altitude;
  auto node = boost::make_shared<ros::NodeHandle>();
  auto relative_pos_sub =
      node->subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1,
                                         boost::bind(&AeroDrone::getRelativeAltitudeCB, this, _1, &relative_altitude));
  ros::Rate rate(1.0);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
