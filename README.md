# Robot Operating System Courses in ITIS (KFU) 2018

## Getting Started

This course is intended for newbies in Robot Operating System Framework.

The most preferred way of getting started with this repo is to create a catkin workspace:
```shell
# workspace will be located at ~/catkin_ws
git clone <repo_url> ~/catkin_ws
```

### Requirements

- Ubuntu 16.04 (Xenial)
- ROS Kinetic ([installation](http://wiki.ros.org/kinetic/Installation/Ubuntu))
- C++ (GNU GCC ver. 5.4+).
- Favorite IDE or Text Editor (e.g. CLion, [QtCreator + ROS plugin](https://ros-industrial.github.io/ros_qtc_plugin/index.html))

### Build & Run

Build (using `catkin_make`):
```shell 
cd ~/catkin_ws && catkin_make [--pkg package_name]

# for bash users
source ~/catkin_ws/devel/setup.bash

# for zsh users
source ~/catkin_ws/devel/setup.zsh
```

To run a node (executable) inside the package:
```shell
# do not forget to start a master node at first
roscore
```

```shell
rosrun <package_name> <node_name> [params ...]
```

### Tests

- Run unit tests:
```shell
# from the ~/catkin_ws/
catkin_make run_tests[_package_name]

# or from the ~/catking_ws/build
make run_tests[_package_name]
```

## Useful resources

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/)
- [How to run tests for specific package](https://answers.ros.org/question/62583/how-do-i-only-run-tests-for-only-one-package/)

## Contribution

This repo is only for educational purposes. You may want to fork it in order to apply some changes.

However, bug reports and enhancement proposals are welcomed.
