#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>

#include "xbox_pad.h"

class TeleopNode {
	private:
		// Publishers
		ros::Publisher cmd_vel_teleop_publisher;
		ros::Publisher mode_auto_publisher;
		ros::Subscriber joy_subscriber;

		ros::Publisher pump_publisher;
		ros::Publisher left_valve_publisher;
		ros::Publisher center_valve_publisher;
		ros::Publisher right_valve_publisher;
		ros::Publisher servo1_publisher;
		ros::Publisher servo2_publisher;

		// Config
		double max_linear_x_velocity;
		double max_linear_y_velocity;
		double max_angular_z_velocity;
		double boost_velocity_ratio;
		std::string joy_topic_name;
		ros::Duration double_click_minimum_interval_;
		ros::Duration double_click_maximum_interval_;
		int32_t servo_increment_;

		// Actual boost ratio
		double current_boost_ratio;

		// State
		double linear_x_, linear_y_, angular_z_;
		bool pump_on_;
		double servo1_pos_;
		bool servo1_up_, servo1_down_;
		double servo2_pos_;
		bool servo2_up_, servo2_down_;

		// Timer
		double rate_;
		boost::asio::io_service service_;
		boost::scoped_ptr<boost::thread> service_thread_;
		boost::asio::io_service::work work_;
		boost::asio::deadline_timer timer_;
		boost::posix_time::time_duration timer_rate_;

		// Buttons state
		std::vector<bool> buttons_last_state_;
		std::vector<ros::Time> buttons_last_pressed_;
		std::vector<ros::Time> buttons_last_release_;
		std::vector<bool> buttons_click_;
		std::vector<bool> buttons_long_click_;
		std::vector<bool> buttons_double_click_;

	public:
		TeleopNode() :
			double_click_minimum_interval_(0.0),
			double_click_maximum_interval_(0.0),

			linear_x_(0.0),
			linear_y_(0.0),
			angular_z_(0.0),

			pump_on_(false),
			servo1_pos_(0.0),
			servo1_up_(false),
			servo1_down_(false),
			servo2_pos_(0.0),
			servo2_up_(false),
			servo2_down_(false),

			service_(),
			service_thread_(),
			work_(service_),
			timer_(service_),

			buttons_last_state_(),
			buttons_last_pressed_(),
			buttons_last_release_(),
			buttons_click_(),
			buttons_long_click_(),
			buttons_double_click_()
		
		{
			ros::NodeHandle nh;
			ros::NodeHandle nh_priv("~");

			double double_click_minimum_interval;
			nh_priv.param("double_click_minimum_interval", double_click_minimum_interval, 0.0);
			double_click_minimum_interval_ = ros::Duration(double_click_minimum_interval);
			double double_click_maximum_interval;
			nh_priv.param("double_click_maximum_interval", double_click_maximum_interval, 0.2);
			double_click_maximum_interval_ = ros::Duration(double_click_maximum_interval);
			nh_priv.param("max_linear_x_velocity", max_linear_x_velocity, 0.1);
			nh_priv.param("max_linear_y_velocity", max_linear_y_velocity, 0.0);
			nh_priv.param("max_angular_z_velocity", max_angular_z_velocity, 1.0);
			nh_priv.param("boost_velocity_ratio", boost_velocity_ratio, 2.0);
			nh_priv.param("joy_topic", joy_topic_name, std::string("joy"));
			nh_priv.param("rate", rate_, 10.0);
			nh_priv.param("servo_increment", servo_increment_, 0);

			timer_rate_ = boost::posix_time::millisec(1000.0/rate_);

			// Start service
			service_thread_.reset(new boost::thread(boost::bind(static_cast<std::size_t(boost::asio::io_service::*)(void)>(&boost::asio::io_service::run), &service_)));

			// Init timer
			timer_.expires_from_now(timer_rate_);
			timer_.async_wait(boost::bind(&TeleopNode::timerCallback, this, _1));
			
			cmd_vel_teleop_publisher = nh_priv.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			mode_auto_publisher = nh.advertise<std_msgs::Bool>("mode_auto", 1);

			pump_publisher = nh.advertise<std_msgs::Int16>("pompe", 1);
			left_valve_publisher = nh.advertise<std_msgs::Int16>("vanne/gauche", 1);
			center_valve_publisher = nh.advertise<std_msgs::Int16>("vanne/milieu", 1);
			right_valve_publisher = nh.advertise<std_msgs::Int16>("vanne/droite", 1);
			servo1_publisher = nh.advertise<std_msgs::Int16>("servo1", 1);
			servo2_publisher = nh.advertise<std_msgs::Int16>("servo2", 1);
			
			joy_subscriber = nh.subscribe<sensor_msgs::Joy>(joy_topic_name, 1, &TeleopNode::joy_handler, this);
		}

		virtual ~TeleopNode()
		{
			// Stop service
			if (service_thread_) {
				service_.stop();
				service_thread_->join();
				service_thread_.reset();
			}

			// Cancel timer
			timer_.cancel();
		}

		void buttonsBehaviourHandler(const sensor_msgs::Joy::ConstPtr& joy)
		{
			// Assign buffers with default values
			if (buttons_last_state_.empty()) {
				buttons_last_state_.assign(joy->buttons.size(), false);
			}
			if (buttons_last_pressed_.empty()) {
				buttons_last_pressed_.assign(joy->buttons.size(), ros::Time(0));
			}
			if (buttons_last_release_.empty()) {
				buttons_last_release_.assign(joy->buttons.size(), ros::Time(0));
			}
			if (buttons_click_.empty()) {
				buttons_click_.assign(joy->buttons.size(), false);
			}
			if (buttons_long_click_.empty()) {
				buttons_long_click_.assign(joy->buttons.size(), false);
			}
			if (buttons_double_click_.empty()) {
				buttons_double_click_.assign(joy->buttons.size(), false);
			}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Beware: we must use our own time base to compute timers as joy stamp is only updated when a button state changes
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			ros::Time now = ros::Time::now();

			for (int i = 0; i < joy->buttons.size(); i++) {
				bool button = joy->buttons[i] != 0;

				// Button is pressed
				if (button) {
					// Up front
					if (!buttons_last_state_[i]) {
						ros::Duration pressed_interval = now - buttons_last_pressed_[i];

						// Double click
						if ((!buttons_last_pressed_[i].is_zero()) &&
								double_click_minimum_interval_ < pressed_interval &&
								pressed_interval < double_click_maximum_interval_) {
							//ROS_INFO("Double click: %d (%f)", i, pressed_interval.toSec());

							buttons_double_click_[i] = true;

							buttons_last_pressed_[i] = ros::Time(0);
						} else {
							buttons_double_click_[i] = false;

							buttons_last_pressed_[i] = now;
						}
					}
				}
				// Button is released
				else {
					// Down front
					if (buttons_last_state_[i]) {
						buttons_last_release_[i] = now;

						if (buttons_long_click_[i]) {
							//ROS_INFO("Long click release: %d (%f)", i, (now - buttons_last_pressed_[i]).toSec());

							buttons_long_click_[i] = false;
						}
					}

					buttons_double_click_[i] = false;
				}

				// Click event
				if (!buttons_last_pressed_[i].is_zero() &&
						(now - buttons_last_pressed_[i]) > double_click_maximum_interval_) {
					/*if (button)
						ROS_INFO("Long click: %d (%f)", i, (now - buttons_last_pressed_[i]).toSec());
					else
						ROS_INFO("Click: %d (%f)", i, (now - buttons_last_pressed_[i]).toSec());*/

					buttons_click_[i] = true;
					buttons_long_click_[i] = button;
					buttons_last_pressed_[i] = ros::Time(0);
				} else {
					buttons_click_[i] = false;
				}

				buttons_last_state_[i] = button;
			}
		}

		void joy_handler(const sensor_msgs::Joy::ConstPtr& joy_msg)
		{
			buttonsBehaviourHandler(joy_msg);

			// STICK TO TWIST
			geometry_msgs::Twist twist;

			current_boost_ratio = 1;
			if(!joy_msg->buttons[XBOX_BUTTON_RB]) {
				current_boost_ratio /= boost_velocity_ratio;
			}

			linear_x_ = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT];
			linear_x_ *= max_linear_x_velocity * current_boost_ratio;

			linear_y_ = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_LEFT];
			linear_y_ *= max_linear_y_velocity * current_boost_ratio;

			angular_z_ = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT];
			angular_z_ *= max_angular_z_velocity * current_boost_ratio;

			// Mode auto
			if (buttons_click_[XBOX_BUTTON_BACK]) {
				std_msgs::Bool bool_msg;
				bool_msg.data = false;
				mode_auto_publisher.publish(bool_msg);
			} else if (buttons_click_[XBOX_BUTTON_START]) {
				std_msgs::Bool bool_msg;
				bool_msg.data = true;
				mode_auto_publisher.publish(bool_msg);
			}

			// Pompe/vannes
			pump_on_ = buttons_click_[XBOX_BUTTON_LB];

			// Servos
			servo1_up_ = buttons_click_[XBOX_BUTTON_Y];
			servo1_down_ = buttons_click_[XBOX_BUTTON_A];
			servo2_up_ = buttons_click_[XBOX_BUTTON_X];
			servo2_down_ = buttons_click_[XBOX_BUTTON_B];
		}

		void timerCallback(const boost::system::error_code& error) {
			// Publish twist at fixed rate
			geometry_msgs::Twist twist;

			twist.linear.x = linear_x_;
			twist.linear.y = linear_y_;
			twist.angular.z = angular_z_;
			
			cmd_vel_teleop_publisher.publish(twist);

			// Pompe/vannes
			if (pump_on_) {
				std_msgs::Int16 msg;
				msg.data = 4096;
				pump_publisher.publish(msg);
				left_valve_publisher.publish(msg);
				center_valve_publisher.publish(msg);
				right_valve_publisher.publish(msg);
			} else {
				std_msgs::Int16 msg;
				msg.data = 0;
				pump_publisher.publish(msg);
				left_valve_publisher.publish(msg);
				center_valve_publisher.publish(msg);
				right_valve_publisher.publish(msg);
			}

			// Servos
			if (servo1_up_)
				servo1_pos_ += servo_increment_ / rate_;
			else if (servo1_down_)
				servo1_pos_ -= servo_increment_ / rate_;

			std_msgs::Int16 servo1_msg;
			servo1_msg.data = servo1_pos_;
			servo1_publisher.publish(servo1_msg);

			if (servo2_up_)
				servo2_pos_ += servo_increment_ / rate_;
			else if (servo2_down_)
				servo2_pos_ -= servo_increment_ / rate_;

			std_msgs::Int16 servo2_msg;
			servo2_msg.data = servo2_pos_;
			servo2_publisher.publish(servo2_msg);
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
