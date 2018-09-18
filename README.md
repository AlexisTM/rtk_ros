ROS RTK node for UBX M8P GPS base station
=========================================

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

