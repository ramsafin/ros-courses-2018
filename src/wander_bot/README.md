# Wander Bot Package (turtlebot)

ROS and Gazebo integration demo.

This package contains two nodes (`stopper` and `forward_rotate_explorer`) which control the __turtlebot__ mobile robot.

The firs node, `stopper`, moves the __turtlebot__ forward (along X-axis) and stops when the laser scan (`/scan` topic) signifies
that there is an obstacle in front of the robot (laser scanner's active range is about 60 degrees).

The other node, `forward_rotate_explorer`, moves the robot forward, in case if there is an obstacle in front it rotates 
to a space that is more free from obstacles and continues to move forward.

## Build & Run

To build this package do the following:
```shell
cd ~/catkin_ws && catkin build wander_bot

source devel/setup.bash
```

Start the `stopper` node (along with Gazebo):
```shell
roslaunch wander_bot stopper.launch
```

Start the `forward_rotate_explorer` node (along with Gazebo):
```shell
roslaunch wander_bot forward_rotate_explorer.launch
```
<p align="center">
  <img src="images/turtlebot in gazebo.png"></img>
</p>

## Misc

Look at the laser scanner's data:
```shell
rostopic echo /scan -n1
```
Start the __turtlebot__ teleoperation node:
```shell
roslaunch turtlebot_teleop keyboard_teleop.launch
```
Visualize lots of interesting sensory data in rviz:
```shell
roslaunch turtlebot_rviz_launchers view_robot.launch
```
