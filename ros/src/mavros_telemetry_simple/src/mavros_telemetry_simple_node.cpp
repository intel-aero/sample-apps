/**
*@file mavros_telemetry_simple_node.cpp
*@brief Demonstration of getting telemetry data of FCU using mavros
*@date 2017-10-24
*/

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <mavros_msgs/BatteryStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

typedef struct
{
  sensor_msgs::NavSatFix global_position_;
  std_msgs::Float64 relative_altitude_;

} GlobalPosition;

typedef struct
{
  geometry_msgs::TwistStamped velocity_param_;
  geometry_msgs::PoseStamped local_param_;
} LocalPosition;

typedef struct
{
  sensor_msgs::BatteryState battery_status_;
} BatteryStatus;

// Printing global position parameters
void getGlobalPosSubCb(const sensor_msgs::NavSatFixConstPtr& msg, bool* global_pos_received, GlobalPosition* global)
{
  // Return if global_pos is already received
  if (*global_pos_received)
    return;

  global->global_position_ = *msg;

  ROS_INFO("\nhome location:\n \t\tlattitude:%lf\n \t\tlongitude:%lf"
           "\n \t\taltitude:%lf\n",
           global->global_position_.latitude, global->global_position_.longitude, global->global_position_.altitude);
  ROS_INFO("Navigation Satellite fix status for any Global Navigation"
           "Satellite System:");

  int status_received = global->global_position_.status.status;
  if (status_received == global->global_position_.status.STATUS_FIX)
    ROS_INFO("status: unaugmented fix");
  else if (status_received == global->global_position_.status.STATUS_NO_FIX)
    ROS_INFO("status: unable to fix position");
  else if (status_received == global->global_position_.status.STATUS_SBAS_FIX)
    ROS_INFO("status: with satellite-based augmentation");
  else
    ROS_INFO("status: with ground-based augmentation");

  int service_received = global->global_position_.status.service;
  if (service_received == global->global_position_.status.SERVICE_GPS)
    ROS_INFO("service: SERVICE_GPS\n\n");
  else if (service_received == global->global_position_.status.SERVICE_GLONASS)
    ROS_INFO("service: SERVICE_GLONASS\n\n");
  else if (service_received == global->global_position_.status.SERVICE_COMPASS)
    ROS_INFO("service: SERVICE_COMPASS\n\n");
  else
    ROS_INFO("service: SERVICE_GALILEO\n\n");

  (*global_pos_received) = true;
}

// Printing relative altitude
void getRelativePosSubCb(const std_msgs::Float64ConstPtr& msg, bool* relative_pos, GlobalPosition* global)
{
  // Return if relative_pos is already received
  if (*relative_pos)
    return;

  global->relative_altitude_ = *msg;
  ROS_INFO("relative altitude:%lf\n", global->relative_altitude_.data);
  *relative_pos = true;
}

// printing local position parameters
void getLocalPosSubCb(const geometry_msgs::PoseStampedConstPtr& msg, bool* local_pos, LocalPosition* local)
{
  // Return if local_pos is already received
  if (*local_pos)
    return;
  local->local_param_ = *msg;
  *local_pos = true;
  ROS_INFO("\nlocal position:\n \t\tlatitude:%lf\n \t\tlongitude:%lf\n"
           "\t\taltitude:%lf\n \t\tw:%lf\n \t\tx:%lf\n \t\ty:%lf\n \t\tz:%lf\n ",
           local->local_param_.pose.position.x, local->local_param_.pose.position.y,
           local->local_param_.pose.position.z, local->local_param_.pose.orientation.w,
           local->local_param_.pose.orientation.x, local->local_param_.pose.orientation.y,
           local->local_param_.pose.orientation.z);
  double quatw = local->local_param_.pose.orientation.w;
  double quatx = local->local_param_.pose.orientation.x;
  double quaty = local->local_param_.pose.orientation.y;
  double quatz = local->local_param_.pose.orientation.z;
  tf::Quaternion q(quatw, quatx, quaty, quatz);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("\nEuler angles:\n \t\troll:%lf\n \t\tyaw:%lf\n \t\tpitch:%lf\n\n", roll, pitch, yaw);
}

// printing battery status
void getBatteryStatusSubCb(const sensor_msgs::BatteryStateConstPtr& msg, bool* battery_pos, BatteryStatus* battery_stat)
{
  // Return if battery_pos is already received
  if (*battery_pos)
    return;
  battery_stat->battery_status_ = *msg;
  *battery_pos = true;
  ROS_INFO("\nbattery parameters:\n \t\tvoltage:%lf\n \t\tcurrent:%lf\n"
           "\t\tcharge:%lf\n \t\tcapacity:%lf\n \t\tdesign_capacity:%lf\n"
           "\t\tpercentage:%lf\n\n",
           battery_stat->battery_status_.voltage, battery_stat->battery_status_.current,
           battery_stat->battery_status_.charge, battery_stat->battery_status_.capacity,
           battery_stat->battery_status_.design_capacity, battery_stat->battery_status_.percentage);
}

// printing velocity parameters
void getVelocitySubCb(const geometry_msgs::TwistStampedConstPtr& msg, bool* velocity_received, LocalPosition* local)
{
  // return if velocity is already received
  if (*velocity_received)
    return;
  local->velocity_param_ = *msg;
  *velocity_received = true;
  ROS_INFO("\nvelocity:\n \tlinear velocity:\n \t\tx:%lf\n \t\ty:%lf\n"
           "\t\tz:%lf\n \tangular velocity:\n \t\tx:%lf\n \t\ty:%lf\n \t\tz:%lf\n\n",
           local->velocity_param_.twist.linear.x, local->velocity_param_.twist.linear.y,
           local->velocity_param_.twist.linear.z, local->velocity_param_.twist.angular.x,
           local->velocity_param_.twist.angular.y, local->velocity_param_.twist.angular.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_telemetry");
  ros::NodeHandle nh;
  LocalPosition local_obj;
  GlobalPosition global_obj;
  BatteryStatus battery_obj;

  bool global_pos_received{};
  bool relative_pos{};
  bool local_pos{};
  bool battery_pos{};
  bool velocity_received{};

  // global position
  ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>(
      "mavros/global_position/global", 10, boost::bind(getGlobalPosSubCb, _1, &global_pos_received, &global_obj));

  ros::Rate rate(20.0);
  while (ros::ok() && !global_pos_received)
  {
    ros::spinOnce();
    rate.sleep();
  }
  global_pos_sub.shutdown();

  // relative altitude
  ros::Subscriber relative_pos_sub = nh.subscribe<std_msgs::Float64>(
      "mavros/global_position/rel_alt", 10, boost::bind(getRelativePosSubCb, _1, &relative_pos, &global_obj));

  while (ros::ok() && !relative_pos)
  {
    ros::spinOnce();
    rate.sleep();
  }
  relative_pos_sub.shutdown();

  // local position
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "mavros/local_position/pose", 10, boost::bind(getLocalPosSubCb, _1, &local_pos, &local_obj));

  while (ros::ok() && !local_pos)

  {
    ros::spinOnce();
    rate.sleep();
  }
  local_pos_sub.shutdown();

  // battery
  ros::Subscriber battery_status__sub = nh.subscribe<sensor_msgs::BatteryState>(
      "mavros/battery", 10, boost::bind(getBatteryStatusSubCb, _1, &battery_pos, &battery_obj));

  while (ros::ok() && !battery_pos)
  {
    ros::spinOnce();
    rate.sleep();
  }
  battery_status__sub.shutdown();

  // velocity
  ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      "mavros/local_position/velocity", 10, boost::bind(getVelocitySubCb, _1, &velocity_received, &local_obj));

  while (ros::ok() && !velocity_received)
  {
    ros::spinOnce();
    rate.sleep();
  }
  velocity_sub.shutdown();

  ROS_INFO("Done\n");
  ros::shutdown();
  return 0;
}
