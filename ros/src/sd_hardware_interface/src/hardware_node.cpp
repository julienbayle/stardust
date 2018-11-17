#include <sd_hardware_interface/hardware_interface.h>
#include <sd_hardware_interface/control_loop.h>
#include <ros/callback_queue.h>

// Main hardware node (starts the controller manager and the hardware interface)
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_interface");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  // Run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();

  // Create the hardware interface
  typedef sd_hardware_interface::HWInterface hw;
  boost::shared_ptr<hw> hw_interface(new hw(nh));
  hw_interface->init();

  // Start the control loop
  sd_hardware_interface::ControlLoop control_loop("control_loop", nh, hw_interface);
  control_loop.run(); // Blocks until shutdown signal recieved

  // Wait for ROS threads to terminate
  ros::waitForShutdown();

  // Release AsyncSpinner object
  spinner.stop();
  
  return 0;
}