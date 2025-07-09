---
title: ROS2 Basics
summary: Basic ROS2 Functionality
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

ROS2 (Robot Operating System 2) is the next-generation open-source framework designed to simplify the development of complex robotic systems. It improves on ROS1 with better performance, support for real-time systems, enhanced security, and cross-platform compatibility. This tutorial will guide you through the fundamental concepts of ROS2, its core components, and how to start developing with it.

By the end of this tutorial, you'll understand the basic architecture of ROS2, how to set up a development environment, create and build packages, and implement simple publisher-subscriber communication between nodes.

---

## ROS2 Architecture

ROS2 is built on a decentralized architecture where processing takes place in nodes. Key components include:

- **Nodes:** Individual executable programs performing computation.
- **Topics:** Named channels for publishing and subscribing to messages.
- **Messages:** Data structures exchanged between nodes.
- **Services:** Synchronous communication for request and response.
- **Parameters:** Configuration values for nodes.

## Setting Up Your Development Environment

Before starting with ROS2, set up your environment. We'll use ROS2 Humble, compatible with Ubuntu 22.04.

1. Install ROS2 by following the official [installation guide](https://docs.ros.org/en/humble/Installation.html).
2. Set up your ROS2 workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/
   colcon build
   source install/setup.bash
   ```

3. Add the source command to your `~/.bashrc` file:

   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Creating a ROS2 Package

In ROS2, software is organized in packages. Create a new package:

### Python

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_package
```

### C++

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_first_package
```

This creates a new ROS2 package named `my_first_package`.

## Writing a Simple Publisher Node

### Python

Create a publisher node that sends a "Hello, ROS2!" message every second.

Edit the `my_first_package/publisher.py` file:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2!'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++

Create a publisher node that sends a "Hello, ROS2!" message every second.

Edit the `my_first_package/publisher.cpp` file:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello, ROS2!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Writing a Simple Subscriber Node

### Python

Create a subscriber node that listens for messages on the 'topic' channel.

Edit the `my_first_package/subscriber.py` file:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++

Create a subscriber node that listens for messages on the 'topic' channel.

Edit the `my_first_package/subscriber.cpp` file:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  void listener_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## Building and Running Your ROS2 Nodes

Build your package and run the nodes:

```bash
cd ~/ros2_ws
colcon build
```

Run the publisher node:

### Python

```bash
ros2 run my_first_package publisher
```

### C++

```bash
ros2 run my_first_package publisher
```

Run the subscriber node:

### Python

```bash
ros2 run my_first_package subscriber
```

### C++

```bash
ros2 run my_first_package subscriber
```
