---
title: ROS2 Rust
summary: Using ROS2 in Rust
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

ROS2 is traditionally used with C++ and Python, but Rust can also be used thanks to community support. Rust is known for its performance, safety, and concurrency support, and so can be desirable for reliable robotics applications. The `rclrs` library enables this integration.

This tutorial will guide you through setting up and using ROS2 in Rust with `rclrs`, including examples of both a publisher and a subscriber node.

## Recommended Pre-Reading

Before reading this, you'll likely want to read the guides on [ROS2 Basics]({{< relref "/post/ros2-basics/" >}}) and [ROS2 Advanced]({{< relref "/post/ros2-advanced/" >}}).

---

## Setting Up Your Environment

Before you begin, ensure you have the necessary tools and dependencies installed on your system.

### Install Rust

If you haven't already, install Rust using `rustup`, which is the recommended toolchain installer for Rust. Open your terminal and run the following command:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

After installation, you can verify that Rust is installed correctly by running:

```bash
rustc --version
```

### Install ROS2

Ensure ROS2 is installed and sourced in your terminal.

## Create a New Rust Project

With Rust and ROS2 installed, you can now create a new Rust project using Cargo, the Rust package manager. Open your terminal and run the following commands:

```bash
cargo new ros2_rust_example
cd ros2_rust_example
```

This will create a new directory called `ros2_rust_example` with a basic Rust project structure.

## Add Dependencies

To use `rclrs` in your Rust project, you need to add it as a dependency. Open the `Cargo.toml` file in your project directory and add the following under the `[dependencies]` section:

```toml
[dependencies]
rclrs = "0.1"  # Check for the latest version on crates.io
```

This will allow your project to use the `rclrs` library for ROS2 integration.

## Write Your ROS2 Node in Rust

### Publisher Example

We can create a simple ROS2 publisher node to publish a string message to a topic every second.

Create a new file named `src/main.rs` and add the following code:

```rust
use rclrs::{Context, Node, Publisher, QoSProfile};
use std::sync::Arc;
use std::time::Duration;

fn main() {
    // Initialise the ROS2 context
    let context = Context::new().unwrap();
    let node = Node::new("rust_node", &context).unwrap();

    // Create a publisher
    let publisher = Publisher::<std_msgs::msg::String>::new(
        &node,
        "topic",
        QoSProfile::default(),
    ).unwrap();

    // Publish a message every second
    loop {
        let msg = std_msgs::msg::String {
            data: "Hello, ROS2 from Rust!".into(),
        };
        publisher.publish(&msg).unwrap();
        std::thread::sleep(Duration::from_secs(1));
    }
}
```

### Subscriber Example

We can also create a ROS2 subscriber node. This node will listen for string messages on a topic and print them to the console.

Modify your `src/main.rs` file as follows:

```rust
use rclrs::{Context, Node, Subscription, QoSProfile};
use std::sync::Arc;

fn main() {
    // Initialise the ROS2 context
    let context = Context::new().unwrap();
    let node = Node::new("rust_subscriber_node", &context).unwrap();

    // Create a subscriber
    let subscription = Subscription::<std_msgs::msg::String>::new(
        &node,
        "topic",
        QoSProfile::default(),
        |msg| {
            println!("Received message: {}", msg.data);
        }
    ).unwrap();

    // Spin to keep the node alive
    loop {
        context.spin_once(std::time::Duration::from_millis(100));
    }
}
```

## Build and Run Your Node

You can now build your project using Cargo. Open your terminal and run the following command:

```bash
cargo build --release
```

This will compile your Rust code and create an executable in the `target/release` directory.

To run your ROS2 node, make sure you have sourced the ROS2 setup script, then execute the following command:

```bash
source /opt/ros/<ros2-distro>/setup.bash
./target/release/ros2_rust_example
```

## Building and Running with `colcon` and `ros2 run`

While you can build and run using Cargo, integrating them into a ROS2 workspace using `colcon` can streamline the development process, especially when working with multiple packages.

### Setting Up Your ROS2 Workspace

**Add Your Rust Package**: Move your Rust project into the `src` directory of your ROS2 workspace.

```bash
mv ~/ros2_rust_example ~/ros2_ws/src/
```

### Modify Your Rust Project for `colcon`

To integrate with `colcon`, you need to make a few modifications:

In the root of your Rust project, create a `CMakeLists.txt` file with the following content:

```cmake
cmake_minimum_required(VERSION 3.5)
project(ros2_rust_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ros2_rust_example src/main.rs)
ament_target_dependencies(ros2_rust_example rclcpp)

install(TARGETS
  ros2_rust_example
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

A `package.xml` file is also required in the project root.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>ros2_rust_example</name>
  <version>0.0.0</version>
  <description>ROS2 Rust Example</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_package</buildtool_depend>

  <depend>rclcpp</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Build & Running

With your Rust project configured for `colcon`, you can now build your entire ROS2 workspace.

```bash
cd ~/ros2_ws
colcon build
```

After building your workspace, you can run your Rust node using the `ros2 run` command. Remember to source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_rust_example ros2_rust_example
```

## Closing Remarks

While this package is promising and provides basic support, it is community-driven and still under active development. Not all ROS2 features are supported yet. For more information, refer to the GitHub for [`rclrs`](https://github.com/ros2-rust/ros2_rust).
