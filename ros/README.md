# Welcome to the Intel Aero ROS-examples guide.

This doc guides how to build, run and create debian packages of ROS-examples on Intel Aero.
## On Ubuntu-based Intel Aero

### Prerequisites
#### ROS
Install ROS-Kinetic by following instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
#### MAVROS
Install [MAVROS](http://wiki.ros.org/mavros) packages as below.
```
$ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script:
```
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ chmod +x install_geographiclib_datasets.sh
$ ./install_geographiclib_datasets.sh
```
#### Catkin tools
Install catkin tools:
```
$ sudo apt-get install python-catkin-tools
```

### Clone and Build ROS-examples
```
$ git clone https://github.com/intel-aero/sample-apps
$ cd ros
$ catkin build
```
To add the workspace to your ROS environment you need to source the generated setup file:
```
$ source devel/setup.bash
```

### Launch MAVROS and Run ROS-examples
MAVROS automatically launches `roscore` which enables communication across ROS nodes.
```
$ roslaunch mavros px4.launch fcu_url:="tcp://<Aero-IP>:5760?ids=1,1"
```
Open another terminal and Run `aero_takeoff_land` example as below:
```
$ roslaunch aero_takeoff_land aero_takeoff_land.launch
```
This successfully launches `aero_takeoff_land.launch`, which connects to Aero flight controller via MAVROS.


## On Docker (Applies to both Ubuntu and Yocto based Intel Aero)
### Prerequisites
Open two terminals (*container-1* and *container-2*) and do below steps in **both**.
#### ROS
Pull ROS packages using docker.
```
$ docker pull ros
```
Run ROS in docker container, this opens a ROS shell with container-id.
```
# docker run -it --privileged ros
```
Update packages.
```
# apt-get update
```
#### MAVROS
Install [MAVROS](http://wiki.ros.org/mavros) packages as below:
```
# apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script:
```
# wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
# chmod +x install_geographiclib_datasets.sh
# ./install_geographiclib_datasets.sh
```
#### Catkin tools
Install catkin tools
```
#  apt-get install python-catkin-tools
```

### Launching MAVROS (In container-1)
#### Export proxy settings
```
export ROS_IP=<IP of container-1> 
```
Docker container IP can be known from `docker network inspect bridge`
```
export ROS_MASTER_URI="http://<IP of container-1>:11311"
```
#### Launch MAVROS
```
# roslaunch mavros px4.launch fcu_url:="tcp://<Aero-IP>:5760?ids=1,1"
```

### Running ROS-examples (In container-2)
#### Clone and Build ROS-examples
```
# git clone https://github.com/intel-aero/sample-apps
# cd ros
# catkin build
```
To add the workspace to your ROS environment you need to source the generated setup file:
```
# source devel/setup.bash
```
#### Export proxy settings
```
export ROS_IP=<IP of container-1>
export ROS_MASTER_URI="http://<IP of container-2>:11311"
```
#### Run `aero_takeoff_land` example as below:
```
# roslaunch aero_takeoff_land aero_takeoff_land.launch
```
*Note: Press`TAB` key to auto-complete the ROS commands.*

#### Running Other Examples: 
##### Fly mission

**Before you run** `aero_fly_mission`:
* Plan your missions in [QGC](http://qgroundcontrol.com) & save them to a file.
Note: `aero_fly_mission` supports only QGC mission plan.

![Qgc_plan](https://user-images.githubusercontent.com/25497245/33707210-b3366a44-db5c-11e7-9165-18c661f01907.png)

Running `aero_fly_mission` example
```
# roslaunch aero_fly_mission aero_fly_mission.launch file:=<absolute path to QGC mission plan>
```

## Creating debian package
### Clone and Build ROS-examples first.
```
$ git clone https://github.com/intel-aero/sample-apps
$ cd ros
$ catkin build
$ source devel/setup.bash
```
Go to particular ROS-example pacakge (for.eg: `aero_takeoff_land`) and build.
```
$ roscd aero_takeoff_land
$ make deb-pkg
```
Debian package will be created in **sample-apps/src/debian-packages/** directory.

## Validation of examples
We have tested on SITL with below test setup:
* PX4 SITL Gazebo multi-robot simulator, version 7.8.1
* QGC v3.2.4
* ROS Kinetic
* MAVROS 0.21.3
