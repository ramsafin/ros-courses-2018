# Chat package (publishers and subscribers)

This package exhibits basic publishing and subscribing approaches in ROS.

There is a **talker** node which publishes messages to the specified topic (`/chat`) 
and **listener** node which subscribes to aforementioned topic.

## Build & Run

Build the `talker` and `listener` nodes:

```shell
cd ~/catkin_ws  # build from catkin workspace

catkin_make --pkg chat_package

source ~/catkin_ws/devel/setup.bash
```

Run the ros master node:

```shell
roscore
```

Start the `talker` node:

```shell
rosrun chat_package talker
```

and `listener` node:

```shell
rosrun chat_package listener
```

### Using launch file

In the `launch` folder you can find `chat.launch`.

Next command will launch two `talker` nodes (**Alice** and **Bob**) and one `listener` node (**EvilHacker**):

```shell
roslaunch chat_package chat.launch
```

In this case you have no need to start ros master node manually, it will be launched automatically.

## Misc

Explore the environment:

```shell
# list all started nodes
rosnode list

# get info about published and subscribed topics 
rosnode info /talker
rosnode info /listener

# show existing topics (there should be at least /chat topic)
rostopic list

# get info about /chat topic
rostopic info /chat

# listen to the messages in /chat
rostopic echo /chat
```
### Rqt Graph

You can visualize the relationship between nodes: topics, publishers and subscribers, etc. in **rqt_graph** node.

After you started ros master node, `talker` and `listener` (you can use a launch file):

```shell
 rosrun rqt_graph rqt_graph
```

<p align="center">
  <image src="pictures/rqt_graph.png" alt="Rqt Graph"></image>
</p>

## Resources

- [ROS Node information](http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information)
- [ROS Callbacks and Spinning](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
- [ROS Publishers and Subscribers](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
