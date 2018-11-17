#ifndef SD_HARDWARE_INTERFACE_ACTUATOR_H
#define SD_HARDWARE_INTERFACE_ACTUATOR_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/scoped_ptr.hpp>

namespace sd_hardware_interface
{

  /// \brief Actuator
  class VelocityController
  {
  public:

    /** \brief Constructor */
    VelocityController(ros::NodeHandle& nh, std::string joint_name, double max_effort, double min_effort);

    /**
    * \brief Get the PID parameters
    */
    void getGains(double &p, double &i, double &d, 
        double &i_max, double &i_min, bool &antiwindup);

    /**
    * \brief Set the PID parameters
    */
    void setGains(
        const double &p, const double &i, const double &d, 
        const double &i_max, const bool &antiwindup);


    /**
    * \brief Run PID
    */
    double velocityToEffort(double joint_velocity, double command_velocity, ros::Duration& period);

    /**
    * \brief Publish regulator data to a control_msgs::JointControllerState
    */
    void publishState();

  private:

    // Max effort value
    double max_effort_;

    // Min effort value
    double min_effort_;

    // Memory for "publishState"
    double last_command_velocity_;
    double last_joint_velocity_;
    ros::Duration last_period_;
    double last_effort_;

    // Internal PID controller
    control_toolbox::Pid pid_controller_; 

    // Publish data to plot PID data
    boost::scoped_ptr<realtime_tools::RealtimePublisher<
        control_msgs::JointControllerState> > controller_state_publisher_ ;
  };

}	// sd_hardware_interface

#endif	// SD_HARDWARE_INTERFACE_MOTOR_ENCODER_H