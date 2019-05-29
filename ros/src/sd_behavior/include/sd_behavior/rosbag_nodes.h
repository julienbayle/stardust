#ifndef SD_ROSBAG_NODES_H
#define SD_ROSBAG_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>


namespace RosbagNodes
{
   	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

	// Async spinner node handle
	static ros::NodeHandle* nh_;

	// Root directory for bag files
	static std::string bags_path_;
	
    class PlayRosbag : public BT::CoroActionNode
	{
		public:
			PlayRosbag(const std::string& name, const BT::NodeConfiguration& config);
			~PlayRosbag();

			BT::NodeStatus tick() override;

			virtual void halt() override;
			
		    	static BT::PortsList providedPorts()
		    	{
			    return { 
				    	BT::InputPort<std::string>("nom"), 
						BT::InputPort<std::string>("topics") 
				};
			}

		private:
			
			std::map<std::string, ros::Publisher> playback_publishers_;
			bool initialized_;
			rosbag::View view_;
			rosbag::Bag bag_;
			ros::Time bag_start_time_;
			ros::Time first_message_time_;
			ros::Time last_message_time_;

			bool is_paused_;
			ros::Time pause_start_time_;
			ros::Duration pause_;

  			boost::mutex rosbag_bag_mtx_;
			std::atomic_bool halted_;
	};	
}
#endif //SD_ROSBAG_NODES_H