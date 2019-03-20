/* Inspired by https://github.com/davetcoleman/ros_control_boilerplate */

#include <sd_hardware_interface/hardware_interface.h>
#include <sd_hardware_interface/control_loop.h>
#include <limits>

// Basic file operations
#include <iostream>
#include <fstream>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace sd_hardware_interface
{
HWInterface::HWInterface(ros::NodeHandle &nh)
  : name_("hardware_interface")
  , nh_(nh)
{
  loadURDF(nh_, "robot_description");

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_); 
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "is_quadrature_encoder", is_quadrature_encoder_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controller_enabled", velocity_controller_enabled_);
  error += !rosparam_shortcuts::get(name_, rpnh, "timeout", timeout_);
  error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
  error += !rosparam_shortcuts::get(name_, rpnh, "encoder_topics", encoder_topics_);
  error += !rosparam_shortcuts::get(name_, rpnh, "pwm_topics", pwm_topics_);
  error += !rosparam_shortcuts::get(name_, rpnh, "speed_topics", speed_topics_);
  error += !rosparam_shortcuts::get(name_, rpnh, "pwm_max", velocity_controllers_pwm_max_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controllers_velocity_to_pwm", velocity_controllers_velocity_to_pwm_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controllers_friction_pwm", velocity_controllers_friction_pwm_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controllers_pid_p", velocity_controllers_pid_p_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controllers_pid_i", velocity_controllers_pid_i_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_controllers_pid_d", velocity_controllers_pid_d_);
  error += !rosparam_shortcuts::get(name_, rpnh, "encoder_steps_for_one_wheel_revolution", encoder_steps_for_one_wheel_revolution_);
  rosparam_shortcuts::shutdownIfError(name_, error);
}

HWInterface::~HWInterface()
{
}

void HWInterface::init()
{
  num_joints_ = joint_names_.size();

  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_effort_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  
  if (num_joints_ != encoder_topics_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "encoder_topics parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != pwm_topics_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "pwm_topics parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != speed_topics_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "speed_topics parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != encoder_steps_for_one_wheel_revolution_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "encoder_steps_for_one_wheel_revolution parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_pwm_max_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "pwm_max parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_velocity_to_pwm_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "velocity_controllers_velocity_to_pwm parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_friction_pwm_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "velocity_controllers_friction_pwm parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_pid_p_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "velocity_controllers_pid_p parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_pid_i_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "velocity_controllers_pid_i parameter must be the same size as joints");
    return;
  }

  if (num_joints_ != velocity_controllers_pid_d_.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "velocity_controllers_pid_d parameter must be the same size as joints");
    return;
  }

  // Initialize interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Initialize joint hardware: " << joint_names_[joint_id]);

    if (urdf_model_ == NULL)
    {
      ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to check joint name");
      return;
    }

    // Get joint from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

    // Get main joint limits
    if (urdf_joint == NULL)
    {
      ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
      return;
    }

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[joint_id], &joint_position_[joint_id], 
        &joint_velocity_[joint_id], &joint_effort_[joint_id]));

    // Add command interface
    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]));
    effort_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]));

    // Hardware related topics
    std::string encoder_topic = encoder_topics_[joint_id];
    std::string pwm_topic = pwm_topics_[joint_id];
    std::string speed_topic = speed_topics_[joint_id];

    boost::shared_ptr<Encoder> joint_encoder( 
      new Encoder(
        encoder_steps_for_one_wheel_revolution_[joint_id],
        timeout_,
        32767.0,
        -32768.0));
    joint_encoders_.push_back(joint_encoder);
    encoder_subscribers_.push_back(nh_.subscribe(encoder_topic, 1, &Encoder::updateFromInt16Topic, joint_encoder));

    boost::shared_ptr<VelocityController> joint_velocity_controller( 
      new VelocityController(
        nh_,
        joint_names_[joint_id],
        velocity_controllers_pwm_max_[joint_id],
        -1.0 * velocity_controllers_pwm_max_[joint_id]));

    // Configure feed forward and friction compensation
    joint_velocity_controller->setFeedforwardAndFrictionGains(
      velocity_controllers_velocity_to_pwm_[joint_id], 
      velocity_controllers_friction_pwm_[joint_id]);
    
    // Limit I to 30% of PWM max
    joint_velocity_controller->setGains(
      velocity_controllers_pid_p_[joint_id],
      velocity_controllers_pid_i_[joint_id],
      velocity_controllers_pid_d_[joint_id],
      0.3 * velocity_controllers_pwm_max_[joint_id], true);

    joint_velocity_controllers_.push_back(joint_velocity_controller);
    effort_publishers_.push_back(nh_.advertise<std_msgs::Int16>(pwm_topic, 1, false));
    speed_publishers_.push_back(nh_.advertise<std_msgs::Int32>(speed_topic, 1, false));

  }  // end for each joint

  registerInterface(&joint_state_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO_STREAM_NAMED(name_, "HWInterface Ready.");
}


void HWInterface::printStatus()
{
  std::stringstream ss;
  ss.precision(6);

  ss << "Joint link" << std::fixed << "\t\t ";
  ss << "Pos(rad)" << std::fixed << "\t ";
  ss << "Vel(rad/s)" << std::fixed << "\t ";
  ss << "Effort(pwm)" << std::fixed << "\t ";
  ss << "Cmd (N/m)" << std::fixed << "\t ";
  ss << "Cmd (rad/s)" << std::fixed << "\t ";
  ss << "Is alive" << std::fixed << std::endl;
  
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << joint_names_[i] << std::fixed << "\t ";;
    ss << std::fixed << joint_position_[i] << "\t ";
    ss << std::fixed << joint_velocity_[i] << "\t ";
    ss << std::fixed << joint_effort_[i] << "\t ";
    ss << std::fixed << joint_effort_command_[i] << "\t ";
    ss << std::fixed << joint_velocity_command_[i] << "\t ";
    ss << std::fixed << joint_encoders_[i]->isAlive() << std::endl;
  }

  ROS_INFO_STREAM_THROTTLE(1, std::endl << ss.str());
}


void HWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    ROS_INFO_STREAM_NAMED(name_, 
      "Waiting for model URDF on the ROS param server at location: " << param_name);
      nh.getParam(param_name, urdf_string);
  
    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  else
    ROS_INFO_STREAM_NAMED(name_, "Received URDF from param server");
}


void HWInterface::read(ros::Duration &elapsed_time)
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    joint_position_[joint_id] = joint_encoders_[joint_id]->getAngle();
    joint_velocity_[joint_id] = joint_encoders_[joint_id]->getSpeed();
  }
}


void HWInterface::write(ros::Duration &elapsed_time)
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) 
  {

    double cmd_effort = joint_effort_command_[joint_id];
    double cmd_speed = 0.0;
    bool security = false;

    if (velocity_controller_enabled_) {
      cmd_effort = joint_velocity_controllers_[joint_id]->velocityToEffort(
          joint_velocity_[joint_id],
          joint_velocity_command_[joint_id],
          elapsed_time);
    } else {
      cmd_speed = joint_velocity_command_[joint_id];
    }
   
    // Safety
    if(!joint_encoders_[joint_id]->isAlive()) {
        ROS_ERROR_STREAM_THROTTLE(1, "Encoder do not emit data. By security, actuator control commannd is set to velocicity = 0 (stay on place)");
        cmd_effort = 0.0;
        cmd_speed = 0.0;
	security = true;
    }

    if (!security && cmd_speed > -0.1 && cmd_speed < 0.1) {
      // Send effort command to hardware
      std_msgs::Int16 effort;
      effort.data = cmd_effort;  
      effort_publishers_[joint_id].publish(effort);

      // Set joint effort
      joint_effort_[joint_id] = cmd_effort;

      // Direction (for simple encoders)
      if(!is_quadrature_encoder_)
        joint_encoders_[joint_id]->setDirection(effort_publishers_[joint_id] > 0);
    }
    else {
      // Send velocity to hardware (use hardware internal PID)
      std_msgs::Int32 speed;
      speed.data = cmd_speed;  
      speed_publishers_[joint_id].publish(speed);
    }
  }
}

void HWInterface::publishVelocityControllerState()
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) 
    joint_velocity_controllers_[joint_id]->publishState();
}

}  // namespace
