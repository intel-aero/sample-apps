
#include "aero_drone_flight_modes.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

AeroDrone::AeroDrone()
{
  resetHome();
  getHomeGeoPoint();
}

AeroDrone::~AeroDrone()
{
  delete thread_watch_flight_mode_;
}

bool AeroDrone::arm()
{
  if (!home_set_)
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
  if (!home_set_)
  {
    ROS_ERROR("Can't takeoff: No GPS Fix!");
    return false;
  }

  auto takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff{};

  srv_takeoff.request.altitude = home_.geo.altitude;
  srv_takeoff.request.latitude = home_.geo.latitude;
  srv_takeoff.request.longitude = home_.geo.longitude;

  // Thread that watch for change in Aero flight mode changes.
  thread_watch_flight_mode_ = new boost::thread(boost::bind(&AeroDrone::watchFlightModeThread, this));

  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::land()
{
  if (!home_set_)
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
  home_.geo.latitude = home_.geo.longitude = home_.geo.altitude = NAN;
}

void AeroDrone::getHomeGeoPoint()
{
  // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
  auto home_sub = nh_.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
                                                           boost::bind(&AeroDrone::setHomeGeoPointCB, this, _1));

  ROS_INFO("Waiting for Aero FC Home to be set...");
  while (ros::ok() && !home_set_)
  {
    ros::spinOnce();
    rate_.sleep();
  }
}

// Callback that gets called periodically from MAVROS notifying Global Poistion of Aero FCU
void AeroDrone::setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home)
{
  home_ = *home;
  home_set_ = true;
  ROS_INFO("Received Home (WGS84 datum): %lf, %lf, %lf", home_.geo.latitude, home_.geo.longitude, home_.geo.altitude);
}

void AeroDrone::printFlightModeCB(const mavros_msgs::StateConstPtr& mode)
{
  ROS_INFO("Flightmode: [%s]", mode->mode.c_str());
}

void AeroDrone::watchFlightModeThread()
{
  auto state_sub =
      nh_.subscribe<mavros_msgs::State>("mavros/state", 1, boost::bind(&AeroDrone::printFlightModeCB, this, _1));

  while (ros::ok())
  {
    ros::spinOnce();
    rate_.sleep();
  }
}
