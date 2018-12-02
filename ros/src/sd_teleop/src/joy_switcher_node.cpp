#include "ros/ros.h"

#include "sensor_msgs/Joy.h"

#include "xbox_pad.h"

class JoySwitcherNode {

    public:

        JoySwitcherNode() : 
                publish_r2(false)
        {
            ROS_INFO("Starting joy_switcher_node");

            ros::NodeHandle nh;

            joy_r1_publisher = nh.advertise<sensor_msgs::Joy>("r1/joy", 1);
            joy_r2_publisher = nh.advertise<sensor_msgs::Joy>("r2/joy", 1);

            joy_subscriber = nh.subscribe("joy", 1, &JoySwitcherNode::input_handler, this);
        };
    

        void input_handler(const sensor_msgs::JoyConstPtr& input_msg) {
            if (input_msg->buttons[XBOX_BUTTON_STICK_LEFT]) {
                publish_r2 = false;
                ROS_INFO("Switch telop to robot 1");
            } else if (input_msg->buttons[XBOX_BUTTON_STICK_RIGHT]) {
                publish_r2 = true;
                ROS_INFO("Switch telop to robot 2");
            }

            if (publish_r2) {
                joy_r2_publisher.publish(input_msg);
            } else {
                joy_r1_publisher.publish(input_msg);
            }
        }

        void run() {           
            ros::spin();
        }

    protected:
        ros::Publisher joy_r1_publisher;
        ros::Publisher joy_r2_publisher;

        ros::Subscriber joy_subscriber;

        bool publish_r2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_switcher_node");
    
    JoySwitcherNode tsn;
    tsn.run();
}