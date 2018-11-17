# Stardust robot hardware interface

This package offers an hardware interface for both robots compatible with [ros_control](http://wiki.ros.org/ros_control).

It uses topics and rosserial to communicate with the hardware

## Encoder listener

Listen to encoder updates and compute joint angle and velocities.

Low velocities calculation uses rolling means.

The code is compatible with quadrature and simple encoders.

## Hardware interface

An effort hardware interface (motor) for the robots

## Controller manager

In order to use the controller manager from the command line, you should set the namespace.

Example :

```bash
export ROS_NAMESPACE=/r1
rosrun controller_manager controller_manager list
```