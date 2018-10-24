#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"


class TwistSwitcherNode : ros::NodeHandle {

    public:

        TwistSwitcherNode(const std::string& ns = std::string()) : ros::NodeHandle(ns) {
            ROS_INFO("Starting twist_switcher_node");

            this->param("source_topic", source_topic, source_topic);
            this->param("output_topic", output_topic, output_topic);

            twist_publisher = this->advertise<geometry_msgs::Twist>(output_topic, 1);
            twist_source_subscriber = this->subscribe<std_msgs::String>(source_topic, 
                10, &TwistSwitcherNode::source_topic_handler, this);
        };
    

        void input_handler(const geometry_msgs::Twist::ConstPtr& input_msg) {
            twist_publisher.publish(input_msg);
        }


        void source_topic_handler(const std_msgs::String::ConstPtr& source_msg) {
            std::string new_input_topic = source_msg->data;
            
            // same topic name : nothing to do
            if (twist_subscriber && twist_subscriber.getTopic() == new_input_topic ) {
                return;
            }   
            
            // ros automatically unsubscribe to the other topic (internal reference counter)
            twist_subscriber = this->subscribe<geometry_msgs::Twist>(new_input_topic, 
                10, &TwistSwitcherNode::input_handler, this);
            ROS_INFO("TwistSwitcherNode: %s -> %s", 
                new_input_topic.c_str(), output_topic.c_str());
        }

        void run() {           
            ros::spin();
        }

    protected:

        std::string source_topic;
        std::string output_topic;

        ros::Publisher twist_publisher;

        ros::Subscriber twist_subscriber;
        ros::Subscriber twist_source_subscriber;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_switcher_node");
    
    TwistSwitcherNode tsn("~");
    tsn.run();
}