# Custom message package

This package contains **robot_status_node** executable which publishes [RobotStatus](custom_message/msg/RobotStatus.msg) 
messages to the `/status` topic.

## Parameters

| Parameter     | Description   |    Optional   |  Default value | Type | Values |
| ------------- | ------------- | ------------- | -------------- | ----- | ------------- |
| `durability_level`   | robot's durability level | Yes | average | string| low, average, high |
| `robot_id`   | robot's identification number| Yes | 1 | int| - |

## Build & Run

To build this package:
```shell
catkin_make --pkg custom_message
source devel/setup.sh
```
After this step `RobotStatus.h` message will be generated under `~catkin_ws/devel/include/custom_message` folder.

Start the `robot_status_node`:
```shell
roslaunch custom_message robot_status.launch [robot_id:=<int value> durability_level:=<string value>]
```
`Note!` Source `devel/setup.bash` after messages are generated, otherwise ROS will not be able to see them.

Listen to the `status` topic:
```shell
rostopic echo /status
```
By default the robot's status will change from `is_ok: true` to `is_ok: false` in 45 seconds (for average durability).

## Tests

// TODO

## Resources
- [ROS time ovrview](http://wiki.ros.org/roscpp/Overview/Time)
- [Creating ROS messages Wiki](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
