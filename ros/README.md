# Welcome to the Intel Aero ros-examples guide.

This doc guides how to build and run ros-examples.
## On native ROS

### Prerequisites

#### Ros
Install [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) pacakges

 Update Packages
```
$ sudo apt-get update
```
#### Mavros
Install [MAVROS](http://wiki.ros.org/mavros) packages
```
$ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

```
Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script:
```
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ chmod +x install_geographiclib_datasets.sh
$ ./install_geographiclib_datasets.sh
```

#### Catkin
Install catkin tools
```
$ sudo apt-get install python-catkin-tools
```

### Clone and build ROS-examples
```
$ git clone https://github.intel.com/drones/ros-examples.git
$ cd ros-examples
$ catkin build
```
To add the workspace to your ROS environment you need to source the generated setup file:
```
$ source devel/setup.bash
```


### Launch Mavros and Run ROS-examples
MAVROS automatically launches `roscore` which enables communication across ROS nodes.
```
$ roslaunch mavros px4.launch fcu_url:="tcp://<Aero-IP>:5760?ids=1,1"
```

Open another terminal  and run the ROS launch file
```
$ roslaunch aero_takeoff_land aero_takeoff_land.launch
```
This successfully launches `aero_takeoff_land.launch` launch file, which connects to Aero flight Controller via MAVROS.

## On Docker ROS

### Prerequisites

Open two terminals (docker1 and docker2) and do below steps in **both**.

#### Ros
Pull ros packages using docker
```
$ docker pull ros
```
Run ROS in docker container
```
# docker run -it --privileged ros
```
this opens a ROS shell with container-id

 Update Packages
```
# apt-get update
```
#### Mavros
Install [MAVROS](http://wiki.ros.org/mavros) packages
```
# apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script:
```
# wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
# chmod +x install_geographiclib_datasets.sh
# ./install_geographiclib_datasets.sh
```

### Catkin
Install catkin tools
```
#  apt-get install python-catkin-tools
```


## Launching Mavros (In docker1)

### Export proxy settings
```
export ROS_IP=<IP of docker1> 
```
Ip can be known from `docker network inspect bridge`
```
export ROS_MASTER_URI="http://<IP of docker1>:11311"
```

### Launch Mavros
```
# roslaunch mavros px4.launch fcu_url:="tcp://<Aero-IP>:5760?ids=1,1"
```

## Running ROS-examples (In Docker2)

### Clone and build ROS-examples
```
# git clone https://github.intel.com/drones/ros-examples.git
# cd ros-examples
# catkin build
```

To add the workspace to your ROS environment you need to source the generated setup file:
```
# source devel/setup.bash
```

### Export proxy settings
```
export ROS_IP=<IP of docker1>
export ROS_MASTER_URI="http://<IP of docker1>:11311"
```

### Run ROS application using
```
# roslaunch aero_takeoff_land aero_takeoff_land.launch
```

# Running Other Examples : 
## Fly mission

**Before you run** `aero_fly_mission`:
* Plan your missions in [QGC](http://qgroundcontrol.com) & save them to a file.
Note: `aero_fly_mission` supports only QGC mission plan now.

![Qgc_plan](https://user-images.githubusercontent.com/25497245/33707210-b3366a44-db5c-11e7-9165-18c661f01907.png)

Running `aero_fly_mission` example
```
# roslaunch aero_fly_mission aero_fly_mission.launch file:=<absolute path to QGC mission plan>
```



