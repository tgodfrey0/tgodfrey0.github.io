---
title: ROS1 Launch Files, Parameter Servers & Services
summary: Advanced ROS1 Functionality
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

This tutorial will briefly introduce you to the more advanced concepts of ROS1 such as launch file, parameter servers and services.

## Recommended Pre-Reading

Before reading this, you'll likely want to read the [ROS1 Basics]({{< relref "/post/ros1-basics/" >}}).

---

## Launch Files

Launch files allow you to start multiple nodes at once. Create a file named `talker_listener.launch` in the `launch` directory of your package:

### Python Example

```bash
mkdir ~/catkin_ws/src/my_first_package/launch
cd ~/catkin_ws/src/my_first_package/launch
touch talker_listener.launch
```

Add the following content to the launch file:

```python
#!/usr/bin/env python3
import rospy
import roslaunch

def launch():
    rospy.init_node('launch_node', anonymous=True)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    talker_node = roslaunch.core.Node(
        package='my_first_package',
        node_type='talker.py',
        name='talker',
        output='screen'
    )

    listener_node = roslaunch.core.Node(
        package='my_first_package',
        node_type='listener.py',
        name='listener',
        output='screen'
    )

    launch.launch(talker_node)
    launch.launch(listener_node)
    rospy.spin()

if __name__ == '__main__':
    launch()
```

### XML Example

```xml
<launch>
  <node name="talker" pkg="my_first_package" type="talker.py" output="screen" />
  <node name="listener" pkg="my_first_package" type="listener.py" output="screen" />
</launch>
```

Run the launch file:

```bash
roslaunch my_first_package talker_listener.launch
```

## Parameter Servers

Parameter servers allow nodes to store and retrieve global parameters. Modify the talker node to use a parameter for the publishing rate:

### Python Example

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(rospy.get_param('~rate', 1))  ## Default to 1 Hz if not specified
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

Run the talker with a parameter:

```bash
rosrun my_first_package talker.py _rate:=2
```

## ROS Services

Services allow request/reply interactions between nodes. Create a simple service that adds two numbers:

### Python Example

```python
#!/usr/bin/env python3

import rospy
from my_first_package.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo("Returning [%s + %s = %s]", req.a, req.b, result)
    return AddTwoIntsResponse(result)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

### Service Definition File

Create `srv/AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

Update `CMakeLists.txt` and `package.xml` to include service dependencies.
