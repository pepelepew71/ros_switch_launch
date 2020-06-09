# ros_switch_launch

Based on [roslaunch/API Usage](http://wiki.ros.org/roslaunch/API%20Usage).

It seems the roslaunch package can let you create a node to start/shutdown a launch file dynamically.

> Note: The roslaunch.parent.ROSLaunchParent object can only start running in main process.

## Node

switch

## Parameters

path_file

(std_msgs/String) path of launch file.

## Services

~turn_on

(std_srvs/SetBool.srv) start or shutdown launch file.
