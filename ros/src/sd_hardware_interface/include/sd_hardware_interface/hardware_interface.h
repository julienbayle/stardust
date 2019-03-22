/* Inspired by https://github.com/davetcoleman/ros_control_boilerplate */
#ifndef SD_HARDWARE_INTERFACE_HW_INTERFACE_H
#define SD_HARDWARE_INTERFACE_HW_INTERFACE_H

// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


// Hardware interface
#include <sd_hardware_interface/encoder.h>
#include <sd_hardware_interface/velocity_controller.h>

// Dynamic reconfigure
#include "sd_hardware_interface/PIDConfig.h"

namespace sd_hardware_interface
{

  /// \brief Hardware interface for a robot
  class HWInterface : public hardware_interface::RobotHW
  {
  public:
    
    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     */
    HWInterface(ros::NodeHandle &nh);

    /** \brief Destructor */
    virtual ~HWInterface();

    /** \brief Initialize the hardware interface */
    virtual void init();

    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration &elapsed_time);

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration &elapsed_time);

    /** \brief Helper for debugging a joint's state and command */
    virtual void printStatus();

    /** \brief Helper for settings PID values */
    virtual void publishVelocityControllerState();
    
    // Dynamic reconfigure
    virtual void updateParameters(sd_hardware_interface::PIDConfig &config, uint32_t level);

  protected:

    // Load robot URDF
    virtual void loadURDF(ros::NodeHandle &nh, std::string param_name);

    // Short name of this class
    std::string name_;

    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    // Configuration
    std::vector<std::string> joint_names_;
    std::size_t num_joints_;
    urdf::Model *urdf_model_;

    // States
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Commands
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_velocity_command_;

    // Topics
    std::vector<std::string> encoder_topics_;
    std::vector<std::string> encoder_speed_topics_;
    std::vector<std::string> pwm_topics_;
    std::vector<std::string> speed_topics_;
    std::vector<double> encoder_steps_for_one_wheel_revolution_;
    std::vector<double> encoder_speed_steps_for_one_rad_per_second_;

    // Open loop (feed forward + friction model)
    std::vector<double> velocity_controllers_pwm_max_;
    std::vector<double> velocity_controllers_velocity_to_pwm_;
    std::vector<double> velocity_controllers_friction_pwm_;

    //PID
    std::vector<double> velocity_controllers_pid_p_;
    std::vector<double> velocity_controllers_pid_i_;
    std::vector<double> velocity_controllers_pid_d_;

    std::vector<boost::shared_ptr<Encoder> > joint_encoders_;
    std::vector<boost::shared_ptr<VelocityController> > joint_velocity_controllers_;
    std::vector<ros::Subscriber> encoder_subscribers_;
    std::vector<ros::Subscriber> encoder_speed_subscribers_;
    std::vector<ros::Publisher> effort_publishers_;
    std::vector<ros::Publisher> speed_publishers_;

    // Encoder type
    bool is_quadrature_encoder_;

    // Use torque or speed
    bool velocity_controller_enabled_;

    // Topic read timeout
    double timeout_;

  };  // class

}  // sd_hardware_interface

#endif // SD_HARDWARE_INTERFACE_HW_INTERFACE_H
