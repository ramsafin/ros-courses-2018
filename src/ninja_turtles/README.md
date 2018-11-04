# Ninja Turtles Package

This package intends to control the turtle provided by the `turtlesim` package.

## Parameters

| Parameter     | Description   |    Optional   |  Default value (Hz) |
| ------------- | ------------- | ------------- | -------------- |
| `moving_pattern` | how turtle will move | yes | `forward_turn` |

By default the turtle will move 1m forward and then turn counter-clockwise for 45 degrees.

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

or it can be done manually:
```shell
roscore # in the separate terminal tab

rosrun turtlesim turtlesim_node # in the separate tab

rosrun ninja_turtles move_turtle
```
## Tests
// TODO

## Misc

Examine `turtlesim` published and subscribed topics messages:
```shell
rosmsg show turtlesim/Pose # turtles current position

rosmsg show geometry_msgs/Twist # move command
```

Try to move the turtle using the keyboad:
```shell
rosrun turtlesim turtle_teleop_key
```

Subscribe to the turtle's pose (`turtle1/pose` topic):
```shell
rostopic echo /turtle1/pose
```

## Resources
- [How to make subscriber callback function?](https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998#post-id-63998)
- [Turtlesim ROS package official wiki](http://wiki.ros.org/turtlesim)
- [ROS best practices git porject](https://github.com/leggedrobotics/ros_best_practices)
