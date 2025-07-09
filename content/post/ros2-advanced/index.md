---
title: ROS2 Advanced Concepts
summary: Advanced ROS2 Functionality
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

This tutorial will briefly introduce you to the more advanced concepts of ROS2 such as launch file, services and alternative middleware implementations.

## Recommended Pre-Reading

Before reading this, you'll likely want to read the [ROS2 Basics]({{< relref "/post/ros2-basics/" >}}).

---

## Launch Files

Launch files simplify starting multiple nodes. Create a launch file in your package's `launch` directory:

```bash
mkdir ~/ros2_ws/src/my_first_package/launch
nano ~/ros2_ws/src/my_first_package/launch/example.launch.py
```

Add this content:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='publisher',
            output='screen'
        ),
        Node(
            package='my_first_package',
            executable='subscriber',
            output='screen'
        )
    ])
```

Run the launch file:

```bash
ros2 launch my_first_package example.launch.py
```

## Using Services in ROS2

Services in ROS2 allow for request-response communication between nodes. Here's how to create a service:

### Python

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

Define a service in your package:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service = AddTwoIntsService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++

```cpp
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class AddTwoIntsService : public rclcpp::Node
{
public:
  AddTwoIntsService() : Node("add_two_ints_service")
  {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(&AddTwoIntsService::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void add_two_ints_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Received: %d + %d = %d", request->a, request->b, response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsService>());
  rclcpp::shutdown();
  return 0;
}
```

## Alternative ROS Middleware

ROS2 supports multiple DDS (Data Distribution Service) implementations. Some common DDS implementations include:

- **Fast-DDS:** Default middleware for ROS2.
- **RTI Connext:** A commercial DDS implementation with high performance.
- **Cyclone DDS:** A lightweight and open-source implementation.

To switch middleware, set the `RMW_IMPLEMENTATION` environment variable:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

For more advanced middleware configurations, you can provide an XML configuration file, such as `cyclonedds.xml`, to customize settings like participant QoS, logging, and network settings. Place the configuration file in the working directory or specify its path with the `CYCLONEDDS_URI` environment variable:

```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

Refer to the Cyclone DDS documentation for details on creating a valid `cyclonedds.xml` file.
