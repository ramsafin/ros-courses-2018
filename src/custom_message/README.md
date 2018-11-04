# Custom message package

This package contains **robot_status_node** executable which publishes [RobotStatus](custom_message/msg/RobotStatus.msg) 
messages to the `/status` topic.

## Parameters

| Parameter     | Description   |    Optional   |  Default value | Type |
| ------------- | ------------- | ------------- | -------------- | ----- |
| `durability_level`   | robot's durability level | Yes | average | string|
| `robot_id`   | robot's identification number| Yes | 1 | int|

## Build & Run

To build this package:
```shell
catkin_make --pkg custom_message
source devel/setup.sh
```
After this step `RobotStatus` message will be generated under `/devel/include/custom_message` folder (`RobotStatus.h`).

Start the `robot_status_node`:
```shell
roslaunch custom_message robot_status.launch [robot_id:=<int value> durability_level:=<string value>]
```

## Tests

// TODO

## Resources
- [ROS time ovrview](http://wiki.ros.org/roscpp/Overview/Time)
- [Creating ROS messages Wiki] (http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
