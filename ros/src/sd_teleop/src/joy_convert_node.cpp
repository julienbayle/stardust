#include "ros/ros.h"

#include "ps_pad.h"
#include "xbox_pad.h"

#include "sensor_msgs/Joy.h"

namespace sd_teleop {
class JoyConvert {
private:
    ros::NodeHandle nh_, nh_priv_;

	ros::Subscriber ps_joy_subscriber_;
    ros::Publisher joy_publisher_;

public:
    JoyConvert() :
        nh_(),
        nh_priv_("~"),

        ps_joy_subscriber_(nh_.subscribe("joy_ps", 1, &JoyConvert::ps_joy_handler, this)),
        joy_publisher_(nh_.advertise<sensor_msgs::Joy> ("joy", 1, false))
    {

    }

    void ps_joy_handler(const sensor_msgs::JoyConstPtr& ps_joy_msg)
    {
        sensor_msgs::Joy joy_msg;

        joy_msg.header = ps_joy_msg->header;

        joy_msg.buttons.resize(15);
        joy_msg.buttons[XBOX_BUTTON_A] = ps_joy_msg->buttons[PS_BUTTON_CROSS];
        joy_msg.buttons[XBOX_BUTTON_B] = ps_joy_msg->buttons[PS_BUTTON_CIRCLE];
        joy_msg.buttons[XBOX_BUTTON_X] = ps_joy_msg->buttons[PS_BUTTON_SQUARE];
        joy_msg.buttons[XBOX_BUTTON_Y] = ps_joy_msg->buttons[PS_BUTTON_TRIANGLE];
        joy_msg.buttons[XBOX_BUTTON_LB] = ps_joy_msg->buttons[PS_BUTTON_L1];
        joy_msg.buttons[XBOX_BUTTON_RB] = ps_joy_msg->buttons[PS_BUTTON_R1];
        joy_msg.buttons[XBOX_BUTTON_BACK] = ps_joy_msg->buttons[PS_BUTTON_SHARE];
        joy_msg.buttons[XBOX_BUTTON_START] = ps_joy_msg->buttons[PS_BUTTON_OPTIONS];
        joy_msg.buttons[XBOX_BUTTON_POWER] = ps_joy_msg->buttons[PS_BUTTON_POWER];
        joy_msg.buttons[XBOX_BUTTON_STICK_LEFT] = ps_joy_msg->buttons[PS_BUTTON_STICK_LEFT];
        joy_msg.buttons[XBOX_BUTTON_STICK_RIGHT] = ps_joy_msg->buttons[PS_BUTTON_STICK_RIGHT];
        joy_msg.buttons[XBOX_BUTTON_CROSS_LEFT] = ps_joy_msg->axes[PS_AXIS_LEFT_RIGHT_CROSS] > 0;
        joy_msg.buttons[XBOX_BUTTON_CROSS_RIGHT] = ps_joy_msg->axes[PS_AXIS_LEFT_RIGHT_CROSS] < 0;
        joy_msg.buttons[XBOX_BUTTON_CROSS_UP] = ps_joy_msg->axes[PS_AXIS_UP_DOWN_CROSS] > 0;
        joy_msg.buttons[XBOX_BUTTON_CROSS_DOWN] = ps_joy_msg->axes[PS_AXIS_UP_DOWN_CROSS] < 0;

        joy_msg.axes.resize(6);
        joy_msg.axes[XBOX_AXIS_LEFT_RIGHT_STICK_LEFT] = ps_joy_msg->axes[PS_AXIS_LEFT_RIGHT_STICK_LEFT];
        joy_msg.axes[XBOX_AXIS_UP_DOWN_STICK_LEFT] = ps_joy_msg->axes[PS_AXIS_UP_DOWN_STICK_LEFT];
        joy_msg.axes[XBOX_AXIS_LT] = ps_joy_msg->axes[PS_AXIS_L2];
        joy_msg.axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT] = ps_joy_msg->axes[PS_AXIS_LEFT_RIGHT_STICK_RIGHT];
        joy_msg.axes[XBOX_AXIS_UP_DOWN_STICK_RIGHT] = ps_joy_msg->axes[PS_AXIS_UP_DOWN_STICK_RIGHT];
        joy_msg.axes[XBOX_AXIS_RT] = ps_joy_msg->axes[PS_AXIS_R2];

        joy_publisher_.publish(joy_msg);
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_convert_node");

    sd_teleop::JoyConvert joy_convert;

    ros::spin();

    return 0;
}