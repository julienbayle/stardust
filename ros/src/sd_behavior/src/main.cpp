#include <sys/stat.h>
#include <string>
#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

#include "sd_behavior/eye_nodes.h"
#include "sd_behavior/laser_nodes.h"
#include "sd_behavior/gripper_nodes.h"
#include "sd_behavior/move_nodes.h"
#include "sd_behavior/score_nodes.h"
#include "sd_behavior/sensors_nodes.h"
#include "sd_behavior/status_nodes.h"
#include "sd_behavior/timer_nodes.h"


using namespace BT;

long GetFileSize(std::string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

bool run(std::string& path, BehaviorTreeFactory& factory) 
{
    
    try {
      auto tree = factory.createTreeFromFile(path);

      // Real time monitoring with Groot
      BT::PublisherZMQ publisher_zmq(tree);

      // This logger saves state changes on file
      BT::FileLogger logger_file(tree, "bt_trace.fbl", 20);

      ROS_INFO("BT is ready");

      // Run behavior tree
      long loaded_size, current_size;
      loaded_size = current_size = GetFileSize(path); 
      while(loaded_size == current_size && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // execute tree
        tree.root_node->executeTick();

        // watch tree file updates
        current_size = GetFileSize(path); 
      }

      return loaded_size != current_size;
    }
    catch(BT::RuntimeError e)
    {
      ROS_ERROR("BT is incorrect (RuntimeError): %s", e.what());
      return true;
    } 
    catch(BT::LogicError e)
    {
      ROS_ERROR("BT is incorrect (LogicError): %s", e.what());
      return true;
    } 
   
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robot_behavior");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
	nh.setCallbackQueue(&queue);

	// Run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(1, &queue);
	spinner.start();

  // Init behavior tree
  BehaviorTreeFactory factory;

  // Load modules
  std::vector< std::string > bt_modules_array;
  nh.param("bt_modules", bt_modules_array, std::vector< std::string >());
  std::set<std::string> bt_modules(begin(bt_modules_array), end(bt_modules_array));

  if(bt_modules.count("EyeNodes"))        EyeNodes::registerNodes(factory, nh);
  if(bt_modules.count("GripperNodes"))    GripperNodes::registerNodes(factory, nh);
  if(bt_modules.count("LaserNodes"))      LaserNodes::registerNodes(factory, nh);
  if(bt_modules.count("MoveNodes"))       MoveNodes::registerNodes(factory, nh);
  if(bt_modules.count("ScoreNodes"))      ScoreNodes::registerNodes(factory, nh);
  if(bt_modules.count("SensorsNodes"))    SensorsNodes::registerNodes(factory, nh);
  if(bt_modules.count("StatusNodes"))     StatusNodes::registerNodes(factory, nh);
  if(bt_modules.count("TimerNodes"))      TimerNodes::registerNodes(factory);

  // Run behavior tree and auto reload tree on file changed
	std::string fn = argv[1];
  while(run(fn, factory))
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

	// Wait for ROS threads to terminate
	ros::waitForShutdown();

	// Release AsyncSpinner object
  spinner.stop();

	return 0;
}