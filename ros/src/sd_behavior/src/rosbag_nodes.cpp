#include "sd_behavior/rosbag_nodes.h"
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

namespace RosbagNodes
{
    void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh) 
    {
        factory.registerNodeType<RosbagNodes::PlayRosbag>("JouerUnRosbag");
        ros::NodeHandle nh_priv("~");
        nh_priv.param("bags_path", bags_path_, std::string(""));
        //if(bags_path_.empty())
        ROS_ERROR("Bag path not defined ! -%s-", bags_path_.c_str());

        nh_ = &nh;
    }

    PlayRosbag::PlayRosbag(
        const std::string& name, 
        const BT::NodeConfiguration& config) 
			: CoroActionNode(name, config),
              initialized_(false),
              playback_publishers_(),
              rosbag_bag_mtx_(),
              bag_(),
              last_message_time_(ros::Time::now())
    { 
        halted_.store(false);
    }

	BT::NodeStatus PlayRosbag::tick()
	{	
        boost::mutex::scoped_lock(rosbag_bag_mtx_);
        
        if ( ! initialized_ )
        {
            std::string filename;
            if( !getInput("nom", filename) )
                throw BT::RuntimeError("nom is missing");
            
            std::string topics;
            std::vector<std::string> rosbag_topics;
            if( !getInput("topics", topics) )
                throw BT::RuntimeError("topics is missing");
            boost::split(rosbag_topics, topics, [](char c){return c == ',';});

            ROS_DEBUG_STREAM_NAMED("PlayRosBag", 
                "Initialisation rosbag player"
                << "- Fichier : " << bags_path_ + filename
                << " - Nb topics " << rosbag_topics.size());

            bag_.open(bags_path_ + filename, rosbag::bagmode::Read);

            ros::Time start_time = view_.getBeginTime();
            ros::Time end_time = ros::TIME_MAX;
            rosbag::TopicQuery topic_query(rosbag_topics);
            view_.addQuery(bag_, topic_query, start_time, end_time);

            initialized_ = true;
        }
        
        if (view_.size() == 0) {    
            ROS_ERROR("No messages to play on specified topics.");
            return BT::NodeStatus::FAILURE;
        }

        BOOST_FOREACH(rosbag::MessageInstance const m, view_) {
            if( ! nh_->ok() || halted_ )
                return  BT::NodeStatus::FAILURE;

            std::map<std::string, ros::Publisher>::iterator it = playback_publishers_.find(m.getTopic());
            if(it == playback_publishers_.end()) {
                ros::AdvertiseOptions advertise_opts(
                    m.getTopic(), 10, m.getMD5Sum(), m.getDataType(),
                    m.getMessageDefinition());

                ros::Publisher publisher = nh_->advertise(advertise_opts);

                playback_publishers_.insert(
                    playback_publishers_.begin(),
                    std::pair<std::string, ros::Publisher>(m.getTopic(), publisher));
            }

            topic_tools::ShapeShifter::ConstPtr s = m.instantiate<topic_tools::ShapeShifter>();
            playback_publishers_[m.getTopic()].publish(s);
            last_message_time_ = m.getTime();
            ROS_WARN_STREAM("publish " 
                << "- Topic: " << m.getTopic()
                << "- Time: " << last_message_time_);
            setStatusRunningAndYield();
        }
        return BT::NodeStatus::SUCCESS;
	}
	
	void PlayRosbag::halt() 
	{
        ROS_WARN_STREAM("Arret");
		halted_.store(true);
	}

}
