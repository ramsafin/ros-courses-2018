# Ninja Turtles Package

The only node in this package, **turtle_move**, intends to control the `turtlesim` turtle by keeping track of published pose messages (`/turtle1/pose`) and publishing move commands (`turtle1/cmd_vel`).

## Parameters

| Parameter     | Description   |    Optional   |  Default value |
| ------------- | ------------- | ------------- | -------------- |
| `moving_pattern` | turtle's moving behaviour / pattern | yes | `forward_turn` |

By default the turtle will move 1 meter forward and then turn counter-clockwise for 45 degrees.

## Build & Run

To build the `move_turtle` node:
```shell
cd ~catkin_ws && catkin_make --pkg ninja_turtles

source devel/setup.bash
```
To start the `move_turtle` node (using launch file):
```shell
roslaunch ninja_turtles move_turtle.launch [moving_pattern:=<value>]
```
This will start `turtlesim` and `move_turtle` nodes.

or it can be done manually:
```shell
roscore # in the separate terminal tab

rosrun turtlesim turtlesim_node # start the turtlesim

rosrun ninja_turtles move_turtle
```
## Tests
// TODO: No tests, sorry

## Misc

Examine `turtlesim` node:
```shell
rosmsg show turtlesim/Pose # turtles current position

rosmsg show geometry_msgs/Twist # move command
```

Subscribe to the turtle's pose (`turtle1/pose` topic):
```shell
rostopic echo /turtle1/pose
```

and move the turtle using the keyboad:
```shell
rosrun turtlesim turtle_teleop_key
```

## Resources
- [How to make subscriber callback function?](https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998#post-id-63998)
- [Turtlesim ROS package official wiki](http://wiki.ros.org/turtlesim)
- [ROS best practices git project](https://github.com/leggedrobotics/ros_best_practices)
