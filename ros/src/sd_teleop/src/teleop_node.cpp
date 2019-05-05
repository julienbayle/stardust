#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <dynamic_reconfigure/server.h>

#include "xbox_pad.h"
#include "sd_teleop/TeleopConfig.h"

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
		double max_linear_x_velocity_;
		double max_linear_y_velocity_;
		double max_angular_z_velocity_;
		double boost_velocity_ratio_;
		std::string joy_topic_name_;
		ros::Duration double_click_minimum_interval_;
		ros::Duration double_click_maximum_interval_;
		int32_t servo_increment_;

		// Actual boost ratio
		double current_boost_ratio;

		// State
		double linear_x_, linear_y_, angular_z_;
		bool pump_on_;
		bool left_valve_on_, center_valve_on_, right_valve_on_;
		double servo1_pos_;
		bool servo1_up_, servo1_down_;
		double servo2_pos_;
		bool servo2_up_, servo2_down_;

		// Timer
		double timer_frequency_;
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

		// Dynamic reconfigure
		dynamic_reconfigure::Server<sd_teleop::TeleopConfig> dynamic_reconfigure_server_;
		dynamic_reconfigure::Server<sd_teleop::TeleopConfig>::CallbackType dynamic_reconfigure_callback_;

	public:
		TeleopNode() :
			double_click_minimum_interval_(0.0),
			double_click_maximum_interval_(0.0),

			linear_x_(0.0),
			linear_y_(0.0),
			angular_z_(0.0),

			pump_on_(false),
			left_valve_on_(false),
			center_valve_on_(false),
			right_valve_on_(false),
			servo1_pos_(90.0),
			servo1_up_(false),
			servo1_down_(false),
			servo2_pos_(90.0),
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
			buttons_double_click_(),

			dynamic_reconfigure_server_(),
			dynamic_reconfigure_callback_(boost::bind(&TeleopNode::dynamicReconfigureCallback, this, _1, _2))
		
		{
			ros::NodeHandle nh;
			ros::NodeHandle nh_priv("~");
;
			nh_priv.param("joy_topic", joy_topic_name_, std::string("joy"));
			nh_priv.param("timer_frequency", timer_frequency_, 10.0);

			cmd_vel_teleop_publisher = nh_priv.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			mode_auto_publisher = nh.advertise<std_msgs::Bool>("mode_auto", 1);

			pump_publisher = nh.advertise<std_msgs::Int16>("pwm/pompe", 1);
			left_valve_publisher = nh.advertise<std_msgs::Int16>("pwm/vanne2", 1);
			center_valve_publisher = nh.advertise<std_msgs::Int16>("pwm/vanne3", 1);
			right_valve_publisher = nh.advertise<std_msgs::Int16>("pwm/vanne1", 1);
			servo1_publisher = nh.advertise<std_msgs::UInt16>("servo/F", 1);
			servo2_publisher = nh.advertise<std_msgs::UInt16>("servo/E", 1);
			
			joy_subscriber = nh.subscribe<sensor_msgs::Joy>(joy_topic_name_, 1, &TeleopNode::joy_handler, this);

        	dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);

			timer_rate_ = boost::posix_time::millisec(1000.0/timer_frequency_);

			// Start service
			service_thread_.reset(new boost::thread(boost::bind(static_cast<std::size_t(boost::asio::io_service::*)(void)>(&boost::asio::io_service::run), &service_)));

			// Init timer
			timer_.expires_from_now(timer_rate_);
			timer_.async_wait(boost::bind(&TeleopNode::timerCallback, this, _1));
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

    	void dynamicReconfigureCallback(sd_teleop::TeleopConfig &config, uint32_t level) {
			double_click_minimum_interval_ = ros::Duration(config.double_click_minimum_interval);
			double_click_maximum_interval_ = ros::Duration(config.double_click_maximum_interval);
			max_linear_x_velocity_ = config.max_linear_x_velocity;
			max_linear_y_velocity_ = config.max_linear_y_velocity;
			max_angular_z_velocity_ = config.max_angular_z_velocity;
			boost_velocity_ratio_ = config.boost_velocity_ratio;
			servo_increment_ = config.servo_increment;
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
				current_boost_ratio /= boost_velocity_ratio_;
			}

			linear_x_ = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT];
			linear_x_ *= max_linear_x_velocity_ * current_boost_ratio;

			linear_y_ = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_LEFT];
			linear_y_ *= max_linear_y_velocity_ * current_boost_ratio;

			angular_z_ = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT];
			angular_z_ *= max_angular_z_velocity_ * current_boost_ratio;

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

			// Vanne gauche
			bool new_left_valve_on = buttons_click_[XBOX_BUTTON_X] || buttons_long_click_[XBOX_BUTTON_X];
			if (!left_valve_on_ && new_left_valve_on) {
				left_valve_on_ = true;
				std_msgs::Int16 msg;
				msg.data = 4096;
				left_valve_publisher.publish(msg);
			} else if (left_valve_on_ && !new_left_valve_on) {
				left_valve_on_ = false;
				std_msgs::Int16 msg;
				msg.data = 0;
				left_valve_publisher.publish(msg);
			}
			// Vanne milieu
			bool new_center_valve_on = buttons_click_[XBOX_BUTTON_A] || buttons_long_click_[XBOX_BUTTON_A];
			if (!center_valve_on_ && new_center_valve_on) {
				center_valve_on_ = true;
				std_msgs::Int16 msg;
				msg.data = 4096;
				center_valve_publisher.publish(msg);
			} else if (center_valve_on_ && !new_center_valve_on) {
				center_valve_on_ = false;
				std_msgs::Int16 msg;
				msg.data = 0;
				center_valve_publisher.publish(msg);
			}
			// Vanne droite
			bool new_right_valve_on = buttons_click_[XBOX_BUTTON_B] || buttons_long_click_[XBOX_BUTTON_B];
			if (!right_valve_on_ && new_right_valve_on) {
				right_valve_on_ = true;
				std_msgs::Int16 msg;
				msg.data = 4096;
				right_valve_publisher.publish(msg);
			} else if (right_valve_on_ && !new_right_valve_on) {
				right_valve_on_ = false;
				std_msgs::Int16 msg;
				msg.data = 0;
				right_valve_publisher.publish(msg);
			}
			// Pompe
			bool new_pump_on = left_valve_on_ || center_valve_on_ || right_valve_on_;
			if (!pump_on_ && new_pump_on) {
				pump_on_ = true;
				std_msgs::Int16 msg;
				msg.data = 4096;
				pump_publisher.publish(msg);
			} else if (pump_on_ && !new_pump_on) {
				pump_on_ = false;
				std_msgs::Int16 msg;
				msg.data = 0;
				pump_publisher.publish(msg);
			}

			// Shutdown Raspberry PI
			if (buttons_long_click_[XBOX_BUTTON_BACK] && buttons_long_click_[XBOX_BUTTON_START]) {
				system("sudo halt");
			} 

			// Servos
			servo1_up_ = buttons_click_[XBOX_BUTTON_CROSS_UP] || buttons_long_click_[XBOX_BUTTON_CROSS_UP];
			servo1_down_ = buttons_click_[XBOX_BUTTON_CROSS_DOWN] || buttons_long_click_[XBOX_BUTTON_CROSS_DOWN];
			servo2_up_ = buttons_click_[XBOX_BUTTON_CROSS_LEFT] || buttons_long_click_[XBOX_BUTTON_CROSS_LEFT];
			servo2_down_ = buttons_click_[XBOX_BUTTON_CROSS_RIGHT] || buttons_long_click_[XBOX_BUTTON_CROSS_RIGHT];
		}

		void timerCallback(const boost::system::error_code& error) {
			// Publish twist at fixed rate
			geometry_msgs::Twist twist;

			twist.linear.x = linear_x_;
			twist.linear.y = linear_y_;
			twist.angular.z = angular_z_;
			
			cmd_vel_teleop_publisher.publish(twist);

			// Servos
			if (servo1_up_)
				servo1_pos_ += servo_increment_ / timer_frequency_;
			else if (servo1_down_)
				servo1_pos_ -= servo_increment_ / timer_frequency_;
			if (servo1_pos_ < 20)
				servo1_pos_ = 20;
			else if (servo1_pos_ > 170)
				servo1_pos_ = 170;

			std_msgs::UInt16 servo1_msg;
			servo1_msg.data = servo1_pos_;
			servo1_publisher.publish(servo1_msg);

			if (servo2_up_)
				servo2_pos_ += servo_increment_ / timer_frequency_;
			else if (servo2_down_)
				servo2_pos_ -= servo_increment_ / timer_frequency_;
			if (servo2_pos_ < 20)
				servo2_pos_ = 20;
			else if (servo2_pos_ > 170)
				servo2_pos_ = 170;

			std_msgs::UInt16 servo2_msg;
			servo2_msg.data = servo2_pos_;
			servo2_publisher.publish(servo2_msg);

			// Trigger timer
			timer_.expires_from_now(timer_rate_);
			timer_.async_wait(boost::bind(&TeleopNode::timerCallback, this, _1));
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
