# Timer package

Timer package contains only one node (`timer_node`) which publishes current time to the standart output (`/rosout`) 
with defined frequency.

## Parameters

| Parameter     | Description   |    Optional   |  Default value (Hz) |
| ------------- | ------------- | ------------- | -------------- |
| `frequency`   | publishing frequency | yes | 2 |


## Build & Run

To build the `timer_node` executable:

```shell
cd ~/catkin_ws  # build from catkin workspace

catkin_make --pkg timer_package

source ~/catkin_ws/devel/setup.bash  # or zsh
```

Now, you can start `timer_node`:

```shell
# do not forget to start ros master node at first (in the separate terminal tab or window)
roscore

# start the timer node (with default frequency)
rosrun timer_package timer_node
```

or, alternatively, you can start it using a launch file:
```shell
roslaunch timer_package timer.launch [timer_frequency:=<value>]
```

## Tests

Run integration tests:
```shell
rostest timer_package timer_frequency.test
```

Start unit tests (gtest):
```shell
catkin_make run_tests_timer_package
```

## Misc

To stop the node press `Ctrl + C`.

Try acquiring adiitional information:

```shell
# list started nodes (you should see /timer_node along with others)
rosnode list

# get info about published and subscribed topics 
rosnode info /timer_node

# show existing topics
rostopic list

# listen to the messages in /rosout
rostopic echo /rosout

# measure the publishing frequency into /rosout
rostopic hz /rosout 
```
## Resources
- [ROS initialization and shutdown](http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown)
- [ROS node handles](http://wiki.ros.org/roscpp/Overview/NodeHandles)
- [ROS node params](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters)
- [ROS launch file param](http://wiki.ros.org/roslaunch/XML/param)
