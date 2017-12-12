/**
*@file aero_flight_modes_node.cpp
*@brief Demonstration of getting Flight modes using mavros
*@date 2017-10-24
*/

#include <cstdlib>
#include "aero_drone_flight_modes.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_flight_modes");
  AeroDrone aero;
  int ret = EXIT_SUCCESS;

  // Arm
  if (!aero.arm())
  {
    ROS_ERROR("Fatal: Arming failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Arm sent");
  }

  // Takeoff
  if (!aero.takeoff())
  {
    ROS_ERROR("Fatal: Takeoff failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Takeoff sent");
  }

  sleep(5);  // Let Aero reach takeoff altitude

  // Land
  if (!aero.land())
  {
    ROS_ERROR("Fatal: Land failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Land sent");
  }

  sleep(5);  // Let Aero land..

  ROS_INFO("Done");
end:
  ros::shutdown();

  return ret;
}
