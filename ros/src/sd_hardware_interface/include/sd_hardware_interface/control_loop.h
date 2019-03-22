/* Inspired by https://github.com/davetcoleman/ros_control_boilerplate */
#ifndef SD_HARDWARE_INTERFACE_HW_LOOP_H
#define SD_HARDWARE_INTERFACE_HW_LOOP_H

#include <time.h>
#include <sd_hardware_interface/hardware_interface.h>
#include <controller_manager/controller_manager.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "sd_hardware_interface/PIDConfig.h"

namespace sd_hardware_interface
{
    // Used to convert seconds elapsed to nanoseconds
    static const double BILLION = 1000000000.0;

    /**
     * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a
     * specified frequency
     *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
     *        See
     * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
     */
    class ControlLoop
    {
    public:
      /**
       * \brief Constructor
       * \param Node name
       * \param NodeHandle
       * \param CallbackQueue
       * \param hardware_interface - the robot-specific hardware interface to be use with your robot
       */
      ControlLoop(
          std::string name,
          ros::NodeHandle& nh,
          boost::shared_ptr<sd_hardware_interface::HWInterface> hardware_interface);

      // Run the control loop (blocking)
      void run();

    protected:

      // Update function called with loop_hz_ rate
      void update();

      // Dynamic reconfigure      
      void updateParameters(sd_hardware_interface::PIDConfig &config, uint32_t level);
      dynamic_reconfigure::Server<sd_hardware_interface::PIDConfig> dynamic_reconfigure_server_;
      dynamic_reconfigure::Server<sd_hardware_interface::PIDConfig>::CallbackType dynamic_reconfigure_callback_;

      // Startup and shutdown of the internal node inside a roscpp program
      ros::NodeHandle nh_;

      // Name of this class
      std::string name_;

      // Settings
      ros::Duration desired_update_period_;
      double cycle_time_error_threshold_;

      // Timing
      ros::Duration elapsed_time_;
      double loop_hz_;
      struct timespec last_time_;
      struct timespec current_time_;

      // Debug mode (print hardware state in console)
      bool print_state_;

      // Settings mode (publish regulator data to topics)
      bool publish_state_;

      /** \brief ROS Controller Manager and Runner
       *
       * This class advertises a ROS interface for loading, unloading, starting, and
       * stopping ros_control-based controllers. It also serializes execution of all
       * running controllers in \ref update.
       */
      boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

      /** \brief Hardware Interface of the robot */
      boost::shared_ptr<sd_hardware_interface::HWInterface> hardware_interface_;

    }; 

}  // sd_hardware_interface

#endif // SD_HARDWARE_INTERFACE_HW_LOOP_H
