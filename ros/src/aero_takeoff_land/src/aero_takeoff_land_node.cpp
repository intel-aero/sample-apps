/**
 * @file aero_takeoff_land_node.cpp
 * @brief Demonstration of Intel Aero takeoff and land using mavros.
 * @date 2017-10-24
*/

#include <cstdlib>
#include "aero_drone.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_takeoff_land");
  AeroDrone aero;

  if (!aero.arm())
    exit(EXIT_FAILURE);

  if (!aero.takeoff())
    exit(EXIT_FAILURE);

  sleep(5);  // Let Aero reach takeoff altitude

  if (!aero.land())
    exit(EXIT_FAILURE);

  sleep(5);  // Let Aero land..

  ROS_INFO("Done");
  ros::shutdown();

  return 0;
}
