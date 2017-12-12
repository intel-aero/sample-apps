#ifndef __AERO_DRONE_H__
#define __AERO_DRONE_H__

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <thread>
#include <mutex>

/**
 * @brief class that fetches Telemetry info of Intel Aero.
 */
class AeroDrone
{
public:
  AeroDrone();
  ~AeroDrone();

  const mavros_msgs::HomePosition& homePosition();
  const sensor_msgs::NavSatFix& gpsFix();
  float relativeAltitude();
  float absoluteAltitude();
  std::string landedState();
  const sensor_msgs::BatteryState& batteryState();

private:
  void setHomePositionCB(const mavros_msgs::HomePositionConstPtr& home);
  void setGPSFixCB(const sensor_msgs::NavSatFixConstPtr& gps_fix);
  void setAltitudeCB(const mavros_msgs::AltitudeConstPtr& alt);
  void setExtendedStateCB(const mavros_msgs::ExtendedStateConstPtr& extended_state);
  void setBatteryStateCB(const sensor_msgs::BatteryStateConstPtr& battery_state);

  void fetchTelemetryThread();

  ros::NodeHandle nh_;
  ros::Rate rate_ = ros::Rate(10.0);

  std::mutex mutex_{};
  boost::thread* thread_telemetry_ = nullptr;  // for watching drone's altitude

  enum class LandedState
  {
    UNDEFINED,
    ON_GROUND,
    IN_AIR,
    TAKEOFF,
    LANDING
  };

  mavros_msgs::HomePosition home_pos_{};
  sensor_msgs::NavSatFix gps_fix_{};
  mavros_msgs::Altitude altitude{};
  mavros_msgs::ExtendedState extended_state_{};
  sensor_msgs::BatteryState battery_state_{};
};

#endif
