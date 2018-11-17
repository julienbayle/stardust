/* Inspired by https://github.com/davetcoleman/ros_control_boilerplate */

#include <sd_hardware_interface/control_loop.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace sd_hardware_interface
{
  ControlLoop::ControlLoop(
    std::string name,
    ros::NodeHandle& nh,
    boost::shared_ptr<sd_hardware_interface::HWInterface> hardware_interface): 
        nh_(nh), 
        name_(name),
        hardware_interface_(hardware_interface)
  {
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

    // Load rosparams
    std::size_t error = 0;
    ros::NodeHandle rpnh(nh_, name_); 
    error += !rosparam_shortcuts::get(name_, rpnh, "loop_hz", loop_hz_);
    error += !rosparam_shortcuts::get(name_, rpnh, "print_state", print_state_);
    error += !rosparam_shortcuts::get(name_, rpnh, "publish_state", publish_state_);
    error += !rosparam_shortcuts::get(name_, rpnh, "cycle_time_error_threshold", cycle_time_error_threshold_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Get current time for use with first update
    clock_gettime(CLOCK_MONOTONIC, &last_time_);

    desired_update_period_ = ros::Duration(1 / loop_hz_);
  }

  void ControlLoop::run()
  {
    ros::Rate rate(loop_hz_);
    while(ros::ok()) {
      update();
      rate.sleep();
    }
  }

  void ControlLoop::update()
  {
    // Get change in time
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    elapsed_time_ = ros::Duration(
      current_time_.tv_sec - last_time_.tv_sec + 
      (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);

    last_time_ = current_time_;
   
    // Error check cycle time
    const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
    if (cycle_time_error > cycle_time_error_threshold_)
    {
      ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                       << cycle_time_error << ", cycle time: " << elapsed_time_
                                       << ", threshold: " << cycle_time_error_threshold_);
    }

    // Input
    hardware_interface_->read(elapsed_time_);

    // Control
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    hardware_interface_->write(elapsed_time_);

    // Helpers
    if (print_state_)
      hardware_interface_->printStatus();

    if (publish_state_)
      hardware_interface_->publishVelocityControllerState();
  }

}  // sd_hardware_interface