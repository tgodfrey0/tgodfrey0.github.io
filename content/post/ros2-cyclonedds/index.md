---
title: ROS2 with CycloneDDS
summary: ROS2 with the CycloneDDS middleware
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - ROS
---

A brief discussion on using the CycloneDDS middleware with ROS2.

## Recommended Pre-Reading

Before reading this, you'll likely want to read the guides on [ROS2 Basics]({{< relref "/post/ros2-basics/" >}}) and [ROS2 Advanced]({{< relref "/post/ros2-advanced/" >}}).

---

## Brief

ROS2 supports multiple DDS (Data Distribution Service) implementations. Some common DDS implementations include:

- **Fast-DDS**: Default middleware for ROS2.
- **RTI Connext**: A commercial DDS implementation with high performance.
- **Cyclone DDS**: A lightweight and open-source implementation.

This tutorial focuses on using CycloneDDS due to the following reasons:

1. Many commercial robotic systems (e.g., the Go2 from Unitree Robotics).
2. My past experience with it.

## Installation

CycloneDDS is not bundled with ROS2, so we must install it separately.

### Using a Package Manager

We can easily install the CycloneDDS ROS2 middleware using your system's package manager. For systems using `apt`, we can simply run:

```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

This should work in most cases, however, some production robots require specific versions of the package.

### Installing from Source

In order to install a specific version, we can install CycloneDDS from source. First, ensure that ROS2 has not been sourced.

```bash
# Create a CycloneDDS workspace and download sources
mkdir -p ~/cyclonedds_ws/src && cd ~/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b $ROS_DISTRO
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

# Build the package
cd ~/cyclonedds_ws
colcon build --packages-select cyclonedds
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

Before we can use the package, ROS needs to know that CycloneDDS is the desired middleware. This is done through an environment variable.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

It is recommended to set this automatically, for example in `~/.bashrc`.

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

Now that the package has been built and ROS knows to use it, the package needs to be sourced. Similar to the environment variable, it's recommended to source this wherever you source your ROS installation (often in `~/.bashrc`).

```bash
echo "source ~/cyclonedds_ws/install/setup.bash" >> ~/.bashrc
```

ROS should now be using the required version of CycloneDDS as its middleware. To check if it is using the correct DDS system, simply run:

```bash
ros2 doctor --report
```

## Configuration

CycloneDDS allows the specification of the network interface on which to send and receive ROS messages. This is done through the environment variable `CYCLONEDDS_URI`.

We can create the file `~/cyclonedds_ws/cyclonedds.xml` to store our configuration. For example, if other ROS2 nodes were only on the `wlan0` and `eth0` interfaces, a configuration file could force the use of those two interfaces.

```xml
<!-- CycloneDDS Configuration File -->
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlan0" priority="default" multicast="default" />
        <NetworkInterface name="eth0" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

The CycloneDDS environment variable can then be set to point to this file. Similar to the ROS middleware variable, it is recommended to set this in your `~/.bashrc` file.

```bash
echo "export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml" >> ~/.bashrc
```

The environment variable can also be set without creating a separate file by condensing the configuration into one line.

```bash
echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0" priority="default" multicast="default" /><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'" >> ~/.bashrc
```

## Running in a Container

Sometimes you may run into issues with CycloneDDS running in a container, e.g. Docker or Podman. On some systems, when running ROS commands with the `rmw_cyclonedds_cpp` middleware, the command hangs and never returns. To fix this, we must disable multicast in the `cyclonedds.xml` file.

In the CycloneDDS configuration file (or environment variable), add the following tag inside the `<General>` tags.

```xml
<AllowMulticast>spdp</AllowMulticast>
```

## Additional Resources

- [ROS2 Alternative Middleware](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html) - The official ROS2 resources on changing the middleware provider.
- [CycloneDDS Configuration](https://cyclonedds.io/docs/cyclonedds/latest/config/index.html) - Official documentation on how to configure CycloneDDS.
