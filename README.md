ROS RTK node for UBX M8P GPS base station
=========================================

This node is made to work with Mavros's RTK plugin: https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/gps_rtk.cpp

Installation
------------

```bash
mkdir -p $HOME/Workspace/src
cd $HOME/Workspace/src
git clone --recursive http://github.com/AlexisTM/rtk_ros

cd $HOME/Workspace/
catkin build
```

Running
-------

```bash
rosrun rtk_ros rtk_ros_node _port:=/dev/ttyACM0
```

### Parameters

```
port = "/dev/ttyACM0"
baud = 115200
survey/accuracy = 4.0 # meters
survey/duration = 90.0 # seconds
```

### Output

```
~/rtcm_out as mavros_msgs::RTCM # RTCM3 data
~/gps as sensor_msgs::NavSatFix # GPS data
```

To work with mavros, redirect ~/rtcm_out to ~/send_rtcm. 
Once the survey is done, mavros will publish ~/rtk_baseline.
