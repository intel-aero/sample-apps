/**
 * @file mavros_offboard_velocity_node.cpp
 * @brief MAVROS Offboard Control example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL & jMAVSIM.
 */

#include <cstdlib>
#include "aero_drone_action.h"
bool offboardCtrlNed(AeroDrone aero)
{

  // Send it once before starting offboard, otherwise it will be rejected.
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 100);
  ROS_INFO("Done with zero velocity set");

  if (aero.setOffboardMode())
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("unable to switch to offboard");
    return false;
  }

  // Moves up in step pattern

  ROS_INFO("NED: Turn to face south");
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 180.0f, 200);

  ROS_INFO("NED: Move south");
  aero.setOffboardVelocityNed(0.0f, -2.0f, 0.0f, 180.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go up");
  aero.setOffboardVelocityNed(0.0f, 0.0f, 2.0f, 180.0f, 100);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move south");
  aero.setOffboardVelocityNed(0.0f, -2.0f, 0.0f, 180.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go up");
  aero.setOffboardVelocityNed(0.0f, 0.0f, 2.0f, 180.0f, 100);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move south");
  aero.setOffboardVelocityNed(0.0f, -2.0f, 0.0f, 180.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 180.0f, 20);

  ROS_INFO("NED: Go up");
  aero.setOffboardVelocityNed(0.0f, 0.0f, 2.0f, 180.0f, 100);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  // Move  down in step pattern

  ROS_INFO("NED: Turn north");
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 360.0f, 200);

  ROS_INFO("NED: Move north");
  aero.setOffboardVelocityNed(0.0f, 2.0f, 0.0f, 360.0f, 300);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go  down");
  aero.setOffboardVelocityNed(0.0f, 0.0f, -2.0f, 360.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move north");
  aero.setOffboardVelocityNed(0.0f, 2.0f, 0.0f, 360.0f, 300);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go down ");
  aero.setOffboardVelocityNed(0.0f, 0.0f, -2.0f, 360.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move north");
  aero.setOffboardVelocityNed(0.0f, 2.0f, 0.0f, 360.0f, 300);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go down");
  aero.setOffboardVelocityNed(0.0f, 0.0f, -2.0f, 360.0f, 200);

  //Wait for a bit
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 20);
  return true;
}

bool offboardCtrlBody(AeroDrone aero)
{
  ROS_INFO("Started body velocity ");

  // Send it once before starting offboard, otherwise it will be rejected.
  aero.setOffboardVelocityNed(0.0f, 0.0f, 0.0f, 0.0f, 100);
  ROS_INFO("Done with zero velocity set");

  if (aero.setOffboardMode())
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("unable to switch to offboard");
    return false;
  }
  ROS_INFO("Body: Turn around yaw clockwise and climb");
  aero.setOffboardVelocityBody(0.0f, 0.0f, 1.0f, -60.0f, 400);

  ROS_INFO("Body: Turn yaw anti-clockwise");
  aero.setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 60.0f, 400);

  ROS_INFO("Body: Wait for a bit");
  aero.setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f, 100);

  ROS_INFO("Body: Fly a circle");
  aero.setOffboardVelocityBody(5.0f, 0.0f, 0.0f, -30.0f, 1000);

  ROS_INFO("body: Wait for a bit");
  aero.setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f, 200);

  ROS_INFO("Body: Fly a circle sideways");
  aero.setOffboardVelocityBody(0.0f, 5.0f, 0.0f, -30.0f, 1000);

  ROS_INFO("body: Wait for a bit");
  aero.setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f, 200);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  AeroDrone aero;
  int ret = EXIT_SUCCESS;
  bool ret1;

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

  // using local NED co-ordinates
  ret1 = offboardCtrlNed(aero);
  if (ret1 == true)
  {
    ROS_INFO("Done with Ned velocity\n");
  }

  // using body co-ordinates
  ret1 = offboardCtrlBody(aero);
  if (ret1 == true)
  {
    ROS_INFO("Done with body velocity");
  }

  // land
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
  sleep(5);
end:
  ros::shutdown();
  return 0;
}
