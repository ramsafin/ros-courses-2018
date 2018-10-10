# Timer package

Timer package contains only one node (`timer_node`) which prints out current time to the standart output (`/rosout`) 
with defined frequency.

## Parameters

| Parameter     | Description   |    Optional   |  Default value |
| ------------- | ------------- | ------------- | -------------- |
| `frequency`   | printing frequency | yes | 2 Hz|


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

# start the timer node
rosrun timer_package timer_node [frequency]
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
```
