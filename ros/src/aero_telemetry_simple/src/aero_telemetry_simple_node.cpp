/**
*@file aero_telemetry_simple_node.cpp
*@brief Demonstration of getting telemetry data of FCU using mavros
*@date 2017-10-24
*/

#include <aero_drone_telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::this_thread;  // for slee_for()
using namespace std::chrono;       // for seconds()

void printTelemetry(AeroDrone& aero_drone);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_telemetry");
  AeroDrone aero_drone;

  while (ros::ok())
  {
    printTelemetry(aero_drone);
    sleep_for(seconds(1));
  }

  ROS_INFO("Done\n");
  ros::shutdown();
  return 0;
}

void printTelemetry(AeroDrone& aero_drone)
{
  auto home_geo_point = aero_drone.homePosition().geo;
  auto position = aero_drone.gpsFix();
  auto status = aero_drone.gpsFix().status.status;
  auto service = aero_drone.gpsFix().status.service;
  auto battery = aero_drone.batteryState();

  ROS_INFO("---------------------------------------");
  ROS_INFO("Home (WGS84 datum): Lat: %g, Lon: %g, Alt: %g", home_geo_point.latitude, home_geo_point.longitude,
           home_geo_point.altitude);
  ROS_INFO("Absolute altitude (AMSL): %g", aero_drone.absoluteAltitude());
  ROS_INFO("Relative altitude: %g", aero_drone.relativeAltitude());
  ROS_INFO("Position (WGS84 datum): Lat: %g, Lon: %g, Alt: %g", position.latitude, position.longitude,
           position.altitude);

  switch (status)
  {
    case sensor_msgs::NavSatStatus::STATUS_FIX:
      ROS_INFO("Satellite fix: unaugmented fix");
      break;
    case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
      ROS_INFO("Satellite fix: unable to fix position");
      break;
    case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
      ROS_INFO("Satellite fix: satellite-based augmentation");
      break;
    case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
      ROS_INFO("Satellite fix: ground-based augmentation");
      break;
  }

  switch (service)
  {
    case sensor_msgs::NavSatStatus::SERVICE_GPS:
      ROS_INFO("Service: GPS");
      break;
    case sensor_msgs::NavSatStatus::SERVICE_GLONASS:
      ROS_INFO("Service: GLONASS");
      break;
    case sensor_msgs::NavSatStatus::SERVICE_COMPASS:
      ROS_INFO("Service: COMPASS");
      break;
    case sensor_msgs::NavSatStatus::SERVICE_GALILEO:
      ROS_INFO("Service: GALILEO");
      break;
  }
  ROS_INFO("Battery voltage: %gv, remaining: %g%%", battery.voltage, battery.percentage * 100.f);
}
