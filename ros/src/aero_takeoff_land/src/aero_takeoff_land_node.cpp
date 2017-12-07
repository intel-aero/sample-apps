/**
*@file aero_takeoff_land_node.cpp
*@brief Demonstration of takeoff and land using mavros
*@date 2017-10-24
*/

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <string.h>

// printing altitude
void getRelativeAltitudeCb(const std_msgs::Float64ConstPtr& msg, std_msgs::Float64* relative_altitude)
{
  *relative_altitude = *msg;
  ROS_INFO("altitude:%lf", relative_altitude->data);
}

// subscribing to mavros/global_position/rel_alt to get altitude updates
void watchFlightAltitude()
{
  std_msgs::Float64 relative_altitude;
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Subscriber relative_pos_sub = node->subscribe<std_msgs::Float64>(
      "mavros/global_position/rel_alt", 1, boost::bind(getRelativeAltitudeCb, _1, &relative_altitude));
  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_takeoff_land");
  ros::NodeHandle nh;

  // Thread that watch for change in altitude of Aero.
  boost::thread thread_altitude(watchFlightAltitude);
  const float ALTITUDE = 488.00;
  const float LATITUDE = 47.3977415;
  const float LONGITUDE = 8.5455937;
  const float MIN_PITCH = 0;
  const float YAW = 0;

  // arming
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if (arming_client.call(srv_arm) && srv_arm.response.success)
    ROS_INFO("ARM sent %d", srv_arm.response.success);
  else
  {
    ROS_ERROR("Failed arming/disarming");
    ros::shutdown();
    return -1;
  }

  // takeoff
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = ALTITUDE;
  srv_takeoff.request.latitude = LATITUDE;
  srv_takeoff.request.longitude = LONGITUDE;
  srv_takeoff.request.min_pitch = MIN_PITCH;
  srv_takeoff.request.yaw = YAW;
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Takeoff Failed");
    ros::shutdown();
    return -1;
  }
  sleep(5);  // Let Aero reach takeoff altitude

  // land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }

  sleep(5);  // Let Aero land..
  ROS_INFO("Done");
  ros::shutdown();

  return 0;
}
