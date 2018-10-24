#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

#include "xbox_pad.h"

// Publishers
ros::Publisher cmd_vel_teleop_publisher;
ros::Publisher cmd_vel_source_publisher;

// Config
double max_linear_x_velocity;
double max_linear_y_velocity;
double max_angular_z_velocity;
double boost_velocity_ratio;

// Actual boost ratio
double current_boost_ratio;

// Change mode button memory
bool last_change_mode_state = false;

// cmd_vel source index and array
std::vector<std::string> cmd_vel_sources;
int cmd_vel_sources_index = 0;


void joy_handler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	// STICK TO TWIST

	geometry_msgs::Twist twist;

	current_boost_ratio = 1;
	if(!joy_msg->buttons[XBOX_BUTTON_RB]) {
		current_boost_ratio /= boost_velocity_ratio;
	}

	twist.linear.x = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT];
	twist.linear.x *= max_linear_x_velocity * current_boost_ratio;

	twist.linear.y = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_LEFT];
	twist.linear.y *= max_linear_y_velocity * current_boost_ratio;

	twist.angular.z = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT];
	twist.angular.z *= max_angular_z_velocity * current_boost_ratio;
	
	cmd_vel_teleop_publisher.publish(twist);

	// CMD_VEL SOURCE SELECTOR

	if (!joy_msg->buttons[XBOX_BUTTON_START] && last_change_mode_state) {
		cmd_vel_sources_index++;
		if(cmd_vel_sources_index = cmd_vel_sources.size()) {
			cmd_vel_sources_index = 0;
		}
		std_msgs::String cmd_vel_source;
		cmd_vel_source.data = cmd_vel_sources[cmd_vel_sources_index];
		cmd_vel_source_publisher.publish(cmd_vel_source);
		ROS_INFO("Change teleop source to %s", cmd_vel_source.data.c_str());
	}	

	last_change_mode_state = joy_msg->buttons[XBOX_BUTTON_START] > 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sd_teleop");
    ROS_INFO("Starting teleop node");

    ros::NodeHandle nh("~");
    nh.param("max_linear_x_velocity", max_linear_x_velocity, 0.1);
    nh.param("max_linear_y_velocity", max_linear_y_velocity, 0.0);
    nh.param("max_angular_z_velocity", max_angular_z_velocity, 1.0);
    nh.param("boost_velocity_ratio", boost_velocity_ratio, 2.0);
	nh.param("cmd_vel_sources", cmd_vel_sources, cmd_vel_sources);    

    cmd_vel_teleop_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_teleop", 1);
    cmd_vel_source_publisher = nh.advertise<std_msgs::String>("cmd_vel_source", 1, true);
    
    ros::Subscriber joy_subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 1, joy_handler);
    
    ros::spin();
}