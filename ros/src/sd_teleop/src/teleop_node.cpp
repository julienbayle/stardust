#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "xbox_pad.h"

class TeleopNode {
	private:
		// Publishers
		ros::Publisher cmd_vel_teleop_publisher;
		ros::Publisher mode_auto_publisher;
		ros::Subscriber joy_subscriber;

		// Config
		double max_linear_x_velocity;
		double max_linear_y_velocity;
		double max_angular_z_velocity;
		double boost_velocity_ratio;
		std::string joy_topic_name;

		// Actual boost ratio
		double current_boost_ratio;

	public:
		TeleopNode() {
			ros::NodeHandle nh;
			ros::NodeHandle nh_priv("~");

			nh_priv.param("max_linear_x_velocity", max_linear_x_velocity, 0.1);
			nh_priv.param("max_linear_y_velocity", max_linear_y_velocity, 0.0);
			nh_priv.param("max_angular_z_velocity", max_angular_z_velocity, 1.0);
			nh_priv.param("boost_velocity_ratio", boost_velocity_ratio, 2.0);
			nh_priv.param("joy_topic", joy_topic_name, std::string("joy"));
			
			cmd_vel_teleop_publisher = nh_priv.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			mode_auto_publisher = nh.advertise<std_msgs::Bool>("mode_auto", 1);
			
			joy_subscriber = nh.subscribe<sensor_msgs::Joy>(joy_topic_name, 1, &TeleopNode::joy_handler, this);
		}

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

			// Mode auto
			if (joy_msg->buttons[XBOX_BUTTON_BACK]) {
				std_msgs::Bool bool_msg;
				bool_msg.data = false;
				mode_auto_publisher.publish(bool_msg);
			} else if (joy_msg->buttons[XBOX_BUTTON_START]) {
				std_msgs::Bool bool_msg;
				bool_msg.data = true;
				mode_auto_publisher.publish(bool_msg);
			}
		}
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sd_teleop");
    ROS_INFO("Starting teleop node");

	TeleopNode tn;
    
    // Main loop
    ros::spin();
}
