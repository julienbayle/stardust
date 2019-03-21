#include <sd_hardware_interface/encoder.h>

namespace sd_hardware_interface
{
	const double Encoder::BILLION = 1000000000.0;

	Encoder::Encoder(
		double encoder_steps_for_one_joint_revolution,
        	double encoder_speed_steps_for_one_rad_per_second,
		double timeout,
        	double max_position,
        	double min_position)
	  : last_encoder_position_(BILLION + 1)
	  , angle_(0.0)
	  , direction_(1.0)
	  , speed_(0.0)
	  , speed_acc_(RollingWindow::window_size = 10)
	  , timeout_(timeout)
	  , max_position_(max_position)
	  , min_position_(min_position)
	  , encoder_resolution_(6.28318530718 / encoder_steps_for_one_joint_revolution)
	  , encoder_speed_steps_for_one_rad_per_second_(encoder_speed_steps_for_one_rad_per_second)
	{
	}

	void Encoder::updateFromInt16Topic(const std_msgs::Int16ConstPtr &encoder_position)
	{
		update((double) encoder_position->data);
	}

	void Encoder::updateSpeedFromInt32Topic(const std_msgs::Int32ConstPtr &encoder_speed)
	{
    		clock_gettime(CLOCK_MONOTONIC, &current_time_);
    		last_speed_time_ = current_time_;
		
		speed_ = ((double) encoder_speed->data) / encoder_speed_steps_for_one_rad_per_second_;
		speed_acc_(speed_);
	}

	void Encoder::update(double encoder_position)
	{
		// First iteration : Init encoder value and return
		if (!isAlive())
		{
			last_encoder_position_ = encoder_position;
			clock_gettime(CLOCK_MONOTONIC, &last_position_time_);
			ROS_INFO_STREAM("One encoder ready. Initial position: " << last_encoder_position_);
			return;
		}

		double steps = encoder_position - last_encoder_position_;
		
		// Encoder overflows
		if (last_encoder_position_ > max_position_/2 and encoder_position < min_position_/2)	
			steps += max_position_ - min_position_;

		if (last_encoder_position_ < min_position_/2 and encoder_position > max_position_/2)	
			steps -= max_position_ - min_position_;
		
		double last_angle = angle_;
		angle_ += direction_ * steps * encoder_resolution_;

    		clock_gettime(CLOCK_MONOTONIC, &current_time_);
    		last_position_time_ = current_time_;
		last_encoder_position_ = encoder_position;
	}

	void Encoder::setDirection(bool direction) 
	{
		direction_ = direction ? 1.0 : -1.0;
	}

	double Encoder::getAngle() 
	{
		return angle_;
	}

	double Encoder::getSpeed() 
	{
		// Encoder has not yet received any adata
		if (!isAlive())
			return 0.0;

		checkTimeOut(last_speed_time_);
		
		return speed_;
	}

	bool Encoder::checkTimeOut(timespec& last_time)
	{
		clock_gettime(CLOCK_MONOTONIC, &current_time_);
		elapsed_time_ = ros::Duration(
	      		current_time_.tv_sec - last_time.tv_sec + 
	        	(current_time_.tv_nsec - last_time.tv_nsec) / BILLION);
		
		if (elapsed_time_.toSec() > timeout_) {
    		  	ROS_WARN_STREAM("One encoder is down");

    			// set this encoder has dead (timeout)
    			last_encoder_position_ = BILLION+1;

    			// reset speed accumulator
    			speed_acc_ = RollingMeanAcc(RollingWindow::window_size = 10);

    			speed_ = 0.0;
    		}
	}

	double Encoder::getFilteredSpeed()
	{
		// Encoder has not yet received any adata
		if (!isAlive())
			return 0.0;

		checkTimeOut(last_speed_time_);
		
		return bacc::rolling_mean(speed_acc_);
	}

	bool Encoder::isAlive() 
	{
		return last_encoder_position_< BILLION;
	}


} // sd_hardware_interface
