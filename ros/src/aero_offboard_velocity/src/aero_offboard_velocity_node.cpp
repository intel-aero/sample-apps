/**
 * @file mavros_offboard_velocity_node.cpp
 * @brief Aero Offboard Velocity example node, written with MAVROS version: 0.22.0, PX4 FW version: 1.7.3
 * and tested in Gazebo SITL (7.9.0) & jMAVSIM.
 */

#include <cstdlib>
#include "aero_drone_offboard.h"

bool offboardVelocityCtrlNED(AeroDrone& aero);
bool offboardVelocityCtrlBody(AeroDrone& aero);
void stepUp(AeroDrone& aero);
void stepDown(AeroDrone& aero);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  AeroDrone aero;
  int ret = EXIT_SUCCESS;
  bool is_success;

  // Arm
  if (!aero.arm())
  {
    ROS_ERROR("Fatal: Arming failed!");
    ret = EXIT_FAILURE;
    goto end;
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

  // Using local NED co-ordinates
  is_success = offboardVelocityCtrlNED(aero);
  if (is_success)
  {
    ROS_INFO("Offboard velocity control using NED: Done.\n");
  }
  else
  {
    ROS_ERROR("Offboard velocity control using NED: Failed.\n");
  }

  // Using body co-ordinates
  is_success = offboardVelocityCtrlBody(aero);
  if (is_success)
  {
    ROS_INFO("Offboard velocity control using Body: Done.\n");
  }
  else
  {
    ROS_ERROR("Offboard Velocity Control using Body: Failed.\n ");
  }

  // Land
  if (!aero.land())
  {
    ROS_ERROR("Fatal: Land failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  sleep(5);  // Let it land

end:
  ros::shutdown();
  return ret;
}

bool offboardVelocityCtrlNED(AeroDrone& aero)
{
  // Send once before starting offboard, otherwise it will be rejected.
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f);

  if (aero.setOffboardMode())
    ROS_INFO("Offboard enabled");
  else
  {
    ROS_ERROR("Unable to switch to Offboard");
    return false;
  }

  // Moves up in step pattern
  stepUp(aero);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  // Moves down in step pattern
  stepDown(aero);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  return true;
}

bool offboardVelocityCtrlBody(AeroDrone& aero)
{
  ROS_INFO("Started Body velocity");

  // Send it once before starting offboard, otherwise it will be rejected.
  aero.setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);

  if (aero.setOffboardMode())
    ROS_INFO("Offboard enabled");
  else
  {
    ROS_ERROR("Unable to switch to Offboard");
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

  return true;
}

void stepUp(AeroDrone& aero)
{
  ROS_INFO("NED: Turn to face South");
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 180.0f, 200);

  ROS_INFO("NED: Move towards South");
  aero.setOffboardVelocityNED(0.0f, -2.0f, 0.0f, 180.0f, 200);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go Up");
  aero.setOffboardVelocityNED(0.0f, 0.0f, 2.0f, 180.0f, 100);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move towards South");
  aero.setOffboardVelocityNED(0.0f, -2.0f, 0.0f, 180.0f, 200);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go Up");
  aero.setOffboardVelocityNED(0.0f, 0.0f, 2.0f, 180.0f, 100);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move towards South");
  aero.setOffboardVelocityNED(0.0f, -2.0f, 0.0f, 180.0f, 200);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 180.0f, 20);

  ROS_INFO("NED: Go Up");
  aero.setOffboardVelocityNED(0.0f, 0.0f, 2.0f, 180.0f, 100);
}

void stepDown(AeroDrone& aero)
{
  ROS_INFO("NED: Turn North");
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 360.0f, 200);

  ROS_INFO("NED: Move towards North");
  aero.setOffboardVelocityNED(0.0f, 2.0f, 0.0f, 360.0f, 300);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go Down");
  aero.setOffboardVelocityNED(0.0f, 0.0f, -2.0f, 360.0f, 200);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move towards North");
  aero.setOffboardVelocityNED(0.0f, 2.0f, 0.0f, 360.0f, 300);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go Down ");
  aero.setOffboardVelocityNED(0.0f, 0.0f, -2.0f, 360.0f, 200);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Move towards North");
  aero.setOffboardVelocityNED(0.0f, 2.0f, 0.0f, 360.0f, 300);

  // Wait for a bit
  aero.setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f, 20);

  ROS_INFO("NED: Go Down");
  aero.setOffboardVelocityNED(0.0f, 0.0f, -2.0f, 360.0f, 200);
}
