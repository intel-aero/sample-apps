/**
*@file mavros_telemetry_modes_node.cpp
*@brief Demonstration of getting Flight modes using mavros
*@date 2017-10-24
*/

#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <boost/bind.hpp>
#include <string.h>
#include <pthread.h>
#include <boost/thread/thread.hpp>

// printing flight mode
void getFlightModeSubCb(const mavros_msgs::StateConstPtr& msg, mavros_msgs::State* flight_mode)
{
  *flight_mode = *msg;
  ROS_INFO("mode: %s", flight_mode->mode.c_str());
}

// subscribing to mavros/state to get flight mode updates
void watchFlightMode()
{
  mavros_msgs::State flight_mode;
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Subscriber flight_mode_sub =
      node->subscribe<mavros_msgs::State>("mavros/state", 3, boost::bind(getFlightModeSubCb, _1, &flight_mode));
  while (ros::ok())
  {
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_takeoff");
  ros::NodeHandle nh;
  boost::thread thread_mode(watchFlightMode);

  const float ALTITUDE = 500;
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
    ROS_INFO("TAKEOFF sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    ros::shutdown();
    return -1;
  }

  sleep(10);

  // land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Failed Land");
    ros::shutdown();
    return -1;
  }

  sleep(5);
  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
