/**
* @file mavros_fly_mission_node.cpp
* @brief Demonstration of Flying Waypoint missions using MAVROS
* @version 0.0.1
* @date 2017-09-06
*/

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

// Boost includes for parsing QGC plan file (JSON)
#include <boost/bind.hpp>
#include <boost/cstdfloat.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <iostream>
#include <set>
#include <string>

// For parsing QGC Waypoint plan (JSON)
namespace bpt = boost::property_tree;

bool g_is_home_gps_set = false;

void loadFromQGCPlan(const std::string &qgc_plan_file, mavros_msgs::WaypointPush *wp_list)
{
  try
  {
    std::ifstream file(qgc_plan_file);
    std::stringstream ss;

    ss << file.rdbuf();
    file.close();

    // Parse QGC plan begins
    //////////////////////////////////////////////////////////
    bpt::ptree mission_pt;
    bpt::read_json(ss, mission_pt);

    // NOTE: Unexpected type while reading values will cause an exception.
    bool first = true;
    // FOREACH mission item in the list
    for (auto &mi : mission_pt.get_child("mission.items"))
    {
      // See http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
      mavros_msgs::Waypoint wp{};
      // we have mission item now
      wp.frame = mi.second.get<int>("frame");
      wp.command = mi.second.get<int>("command");
      wp.autocontinue = mi.second.get<bool>("autoContinue");
      // Only 1st mission item should be set to true.
      wp.is_current = first ? true : false;
      first = false;
      // Parameters
      std::vector<double> params;
      for (auto &p : mi.second.get_child("params"))
        params.push_back(p.second.get<double>(""));
      wp.param1 = params[0];
      wp.param2 = params[1];
      wp.param3 = params[2];
      wp.param4 = params[3];
      wp.x_lat = params[4];
      wp.y_long = params[5];
      wp.z_alt = params[6];
      // Add it to Waypoint List
      wp_list->request.waypoints.push_back(wp);
    }
    //////////////////////////////////////////////////////////
    // Parse QGC plan ends
  }
  catch (std::exception const &e)
  {
    ROS_ERROR("%s", e.what());
    throw;
  }
}

//  Callback that gets called periodically from MAVROS notifying FCU state
void save_current_state_cb(const mavros_msgs::State::ConstPtr &msg, mavros_msgs::State *pCurrentState)
{
  *pCurrentState = *msg;
}

// Callback that gets called periodically from MAVROS notifying Global Poistion of FCU
void set_home_gps_cb(const sensor_msgs::NavSatFixConstPtr &msg, sensor_msgs::NavSatFix *pHomeGps)
{
  *pHomeGps = *msg;
  g_is_home_gps_set = true;
  ROS_INFO("Received Home: %lf, %lf, %lf", pHomeGps->latitude, pHomeGps->longitude, pHomeGps->altitude);
}

int main(int argc, char **argv)
{
  if (argc == 1)
  {
    ROS_ERROR("Usage: rosrun mavros_fly_mission node <QGC waypoint plan>.json");
    return EXIT_FAILURE;
  }
  // Name of this Application node
  ros::init(argc, argv, "mavros_flymission");

  ros::NodeHandle nh;

  // Used for knowing FCU state, whether Armed ? or current flight mode, etc
  mavros_msgs::State current_state{};

  // FCU state subscription: See
  // http://docs.ros.org/api/mavros_msgs/html/msg/State.html
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(save_current_state_cb, _1, &current_state));

  sensor_msgs::NavSatFix home_gps{};

  // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
  ros::Subscriber home_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10,
                                                                      boost::bind(set_home_gps_cb, _1, &home_gps));

  // See http://docs.ros.org/api/mavros_msgs/html/msg/WaypointList.html
  mavros_msgs::WaypointPush wp_list{};

  try
  {
    loadFromQGCPlan(argv[1], &wp_list);
  }
  catch (std::exception const &e)
  {
    // NOTE: QGC waypointplan (JSON) may contain 'params' valueas 'null';
    // in that case we may get execption. Make sure to keep 'params' values to be 0.
    ROS_ERROR("Fatal: error in loading waypoints from the file %s!!", argv[1]);
    abort();
  }

  ros::Rate rate(20.0);

  ROS_INFO("Waiting for FCU Home GPS location to be set...");
  while (ros::ok() && !g_is_home_gps_set)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // we don't need FCU Home position now
  home_gps_sub.shutdown();

  // Send WPs to Vehicle
  ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
  ROS_INFO("Now, Sending WPs to Vehicle...");
  while (ros::ok())
  {
    if (wp_client.call(wp_list))
    {
      if (!wp_list.response.success)
      {
        // Lets wait till we succeed in sending WPs.
        ROS_ERROR("Lets wait till we succeed in sending WPs");
        ros::spinOnce();
        rate.sleep();
      }
      else
      {
        ROS_INFO("WPs sent to Vehicle");
        break;
      }
    }
  }

  // Set to Mission mode
  ROS_INFO("Now, Setting to Mission mode...");
  mavros_msgs::SetMode mission_set_mode;
  mission_set_mode.request.custom_mode = "AUTO.MISSION";
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
  {
    ROS_INFO("In Mission mode now");
    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    if (!current_state.armed)
    {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed...");
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to set Mission mode!");
    ros::shutdown();
  }

  // Lets not overload FCU
  ROS_INFO("Lets pause for 5 secs (to keep FCU at ease...)");
  sleep(5.0);
  ROS_INFO("Resumed. Missions in execution...");
  bool armed = static_cast<int>(current_state.armed);

  // Be in event loop
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    armed = static_cast<int>(current_state.armed);
    if (!armed)
      break;
  }
  ROS_INFO("Mission accomplished!");

  return 0;
}
