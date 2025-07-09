---
title: ROS1 Basics
summary: Basic ROS1 Functionality
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

ROS1 (Robot Operating System 1) is an open-source framework designed to simplify the development of complex robotic systems. It provides a rich set of tools, libraries, and conventions that enable developers to create robust and scalable robot applications. This tutorial will guide you through the fundamental concepts of ROS1, its core components, and how to start developing with it.

---

## ROS1 Architecture

ROS1 is built on a graph-based architecture where processing takes place in nodes that may receive, post, and multiplex sensor data, control, state, planning, actuator, and other messages. The key concepts in ROS1 are:

- **Nodes:** Executable programs that perform computation.
- **Topics:** Named buses over which nodes exchange messages.
- **Messages:** ROS data type used when subscribing or publishing to a topic.
- **Services:** Request/reply interactions between nodes.
- **Master:** Name service for ROS (i.e., helps nodes find each other).
- **Parameter Server:** A shared, multi-variate dictionary that nodes can use to store and retrieve parameters at runtime.

## Setting Up Your Development Environment

Before we start developing with ROS1, we need to set up our environment. We'll use ROS Noetic, which is designed for Ubuntu 20.04.

1. Install ROS Noetic by following the official installation guide.
2. Set up your catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Add the source command to your `~/.bashrc` file to automatically set up your environment for ROS development:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Creating a ROS Package

In ROS, software is organized in packages. Let's create a simple package:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_first_package rospy std_msgs
```

This creates a new package named "my_first_package" with dependencies on rospy (Python client library for ROS) and std_msgs (standard message types).

## Writing a Simple Publisher Node

ROS supports writing packages in either Python or C++.

### Python Example

Create a new file named `talker.py` in the `scripts` directory of your package:

```bash
mkdir ~/catkin_ws/src/my_first_package/scripts
cd ~/catkin_ws/src/my_first_package/scripts
touch talker.py
chmod +x talker.py
```

Add the following code to `talker.py`:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "Hello, ROS! %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### Explanation of the Code

```python
#!/usr/bin/env python3
```

This line specifies the interpreter to use for the script. It ensures the script is run with Python 3.

```python
import rospy
from std_msgs.msg import String
```

- `rospy`: The Python library for ROS, enabling communication between nodes.
- `std_msgs.msg.String`: The ROS message type used for publishing and subscribing to strings.

```python
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
```

- `rospy.Publisher`: Creates a ROS publisher that publishes messages to the `chatter` topic.
- `queue_size=10`: Limits the number of messages queued for delivery to subscribers.

```python
rospy.init_node('talker', anonymous=True)
```

- Initializes the ROS node named `talker`.
- `anonymous=True`: Appends a random identifier to the node name to avoid name conflicts.

```python
rate = rospy.Rate(1)  # 1 Hz
```

- Creates a `Rate` object to enforce a loop frequency of 1 Hz.

```python
while not rospy.is_shutdown():
    hello_str = "Hello, ROS! %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
```

- Loops until the node shuts down.
- `rospy.get_time()`: Retrieves the current ROS time.
- `rospy.loginfo()`: Logs a message to the console and ROS log files.
- `pub.publish(hello_str)`: Publishes the message to the `chatter` topic.
- `rate.sleep()`: Sleeps to maintain the desired loop frequency.

```python
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

- Ensures the `talker` function runs when the script is executed.
- `rospy.ROSInterruptException`: Catches exceptions triggered when the node is interrupted.

### C++ Example

Create a new file named `talker.cpp` in the `src` directory of your package:

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello, ROS! " << ros::Time::now();
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

#### Explanation of the Code

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
```

- Includes the necessary ROS and message type headers.
- `<sstream>`: Provides functionality to construct strings.

```cpp
ros::init(argc, argv, "talker");
ros::NodeHandle nh;
```

- `ros::init`: Initializes the ROS node with the name `talker`.
- `ros::NodeHandle`: Provides communication with the ROS system.

```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
```

- Advertises a topic named `chatter` with message type `std_msgs::String` and a queue size of 10.

```cpp
ros::Rate loop_rate(1);
```

- Sets the loop frequency to 1 Hz.

```cpp
while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello, ROS! " << ros::Time::now();
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
}
```

- Loops until the ROS system is shut down.
- `ros::Time::now()`: Retrieves the current ROS time.
- `ROS_INFO`: Logs a message to the console and ROS log files.
- `pub.publish(msg)`: Publishes the message to the `chatter` topic.
- `ros::spinOnce()`: Processes incoming messages.
- `loop_rate.sleep()`: Sleeps to maintain the desired loop frequency.

```cpp
return 0;
```

- Exits the program when the loop ends.
