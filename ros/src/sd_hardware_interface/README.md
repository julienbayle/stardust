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

## Velocity controller

The model is a PID with velocity feed forward plus friction boost

For each wheel:

Ouput (PWM) = 
* velocity_goal * velocity_controllers_velocity_to_pwm
* + sign(velocity_goal) * velocity_controllers_friction_pwm
* + velocity_controllers_pid_p * (velocity_goal - velocity)
* + velocity_controllers_pid_i * sum_over_time(velocity_goal - velocity) (contribution limited to 30% of max pwm)
* + velocity_controllers_pid_d * d(velocity_goal - velocity)/dt

with sign(velocity_goal) = 
* -1 if velocity_goal lower than 0
* 0 if velocity_goal is 0
* 1 if velocity_goal is greater than 0