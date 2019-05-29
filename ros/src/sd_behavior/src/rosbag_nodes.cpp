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
        if(bags_path_.empty())
            ROS_ERROR("Bag path parameter is missing.");

        nh_ = &nh;
    }

    PlayRosbag::PlayRosbag(
        const std::string& name, 
        const BT::NodeConfiguration& config) 
			: CoroActionNode(name, config),
              initialized_(false),
              playback_publishers_(),
              rosbag_bag_mtx_(),
              pause_(0.0),
              is_paused_(false),
              pause_start_time_(0),
              bag_()
    { 
        halted_.store(false);
    }

    PlayRosbag::~PlayRosbag() {
        bag_.close();

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
            if( !getInput("topics", topics) )
                throw BT::RuntimeError("topics is missing");
            
            std::vector<std::string> rosbag_topics;
            boost::split(rosbag_topics, topics, [](char c){return c == ',';});

            ROS_DEBUG_STREAM_NAMED("PlayRosBag", 
                "Initialisation rosbag player"
                << "- Fichier : " << bags_path_ + filename
                << " - Nb topics " << rosbag_topics.size());

            rosbag::View full_view;
            bag_.open(bags_path_ + filename, rosbag::bagmode::Read);
            full_view.addQuery(bag_);

            bag_start_time_ = full_view.getBeginTime();
            
            rosbag::TopicQuery topic_query(rosbag_topics);
            view_.addQuery(bag_, topic_query, bag_start_time_, ros::TIME_MAX);

            initialized_ = true;
        }
        
        if (view_.size() == 0) {   
            ROS_ERROR("No messages to play on specified topics.");
            return BT::NodeStatus::FAILURE;
        }

        if( ! is_paused_) {
            ROS_DEBUG_STREAM("Start rosbag " 
                    << "- Total duration: " << view_.getEndTime() - view_.getBeginTime());

            first_message_time_ = last_message_time_ = ros::Time::now();
        }

        BOOST_FOREACH(rosbag::MessageInstance const m, view_) {
            // Replay after pause
            if(is_paused_) {
                
                pause_ += ros::Time::now() - pause_start_time_;
                is_paused_ = false;

                ROS_DEBUG_STREAM("Play rosbag after a pause" 
                    << "- Time: " << (ros::Time::now() - first_message_time_) - pause_);
            }

            if( ! nh_->ok()) {
                return  BT::NodeStatus::FAILURE;
            }

            // Declare publishers
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

            // Ignore messages already played before the pause
            if (m.getTime() - bag_start_time_ <= (ros::Time::now() - first_message_time_) - pause_)
            {
                ROS_DEBUG_STREAM("Ignore message after a pause" 
                    << "- Time: " << m.getTime() - bag_start_time_);
            }
            else
            {
                // Wait to respect message original rate
                while (m.getTime() - bag_start_time_ > (ros::Time::now() - first_message_time_) - pause_)
                    setStatusRunningAndYield();

                // Halted means a pause is requested
                if( halted_ ) {
                    halted_.store(false);
                    return  BT::NodeStatus::FAILURE;
                }
                
                topic_tools::ShapeShifter::ConstPtr s = m.instantiate<topic_tools::ShapeShifter>();
                playback_publishers_[m.getTopic()].publish(s);
                ROS_DEBUG_STREAM("publish " 
                    << "- Topic: " << m.getTopic()
                    << "- Time: " << m.getTime() - bag_start_time_);

                last_message_time_ = ros::Time::now();
            }

        }

        pause_ = ros::Duration(0);
        ROS_DEBUG_STREAM("Rosbag - end of file");
        return BT::NodeStatus::SUCCESS;
	}
	
	void PlayRosbag::halt() 
	{
		pause_start_time_ = last_message_time_;
        is_paused_ = true;
        halted_.store(true);
        ROS_DEBUG_STREAM("Pause rosbag" 
            << "- Time: " << (pause_start_time_ - first_message_time_) - pause_);
	}

}
