/**
 * @file aero_offboard_ctrl_node.cpp
 * @brief Aero Offboard Control example node, written with mavros version 0.21.2, px4 flight
 * stack and tested in Gazebo SITL & jMAVSIM.
 * Original source code: MAVROS OFFBOARD example from: https://dev.px4.io/en/ros/mavros_offboard.html
 *
 * This example is summariesed below:
 * 1. Listens for state of the Aero Flight Controller.
 * 2. Set offboard setpoint location before changing OFFBOARD (Otherwise mode switch will be rejected)
 * 3. Switches to OFFBOARD mode.
 * 4. When Aero Flight Controller goes to OFFBOARD mode, we continue publishing setpoint position with altitude of 2
 * meters.
 * 5. We keep receiving Aero FCU state callabck.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// Callback that gets called at a fixed rate reporting FCU state: connection state, armed, mode, etc.
void printStateCB(const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State* state)
{
  *state = *msg;

  bool connected = state->connected;
  bool armed = state->armed;
  std::string mode = state->mode;

  ROS_INFO("******** Received state cb *********");
  ROS_INFO("%sconnected to PX4", connected ? "" : "Not ");
  ROS_INFO("%sarmed", armed ? "" : "Not ");
  ROS_INFO("%s", mode.c_str());
  ROS_INFO("------------------------------------");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  mavros_msgs::State current_state;

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(printStateCB, _1, &current_state));
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while (ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
