#include <sd_hardware_interface/velocity_controller.h>

namespace sd_hardware_interface
{
    VelocityController::VelocityController(
        ros::NodeHandle& nh, 
        std::string joint_name, 
        double max_effort, 
        double min_effort) :
        max_effort_(max_effort), 
        min_effort_(min_effort),
        feedforward_gain_(0.0),
        friction_gain_(0.0),
        pid_controller_()
    {
        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
        (nh, joint_name.append("_state"), 1));
    }

    /**
    * \brief Get the PID parameters
    */
    void VelocityController::getGains(double &p, double &i, double &d, 
        double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    /**
    * \brief Set the PID parameters
    */
    void VelocityController::setGains(
        const double &p, const double &i, const double &d, 
        const double &i_max, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,-i_max,antiwindup);
    }

    /**
    * \brief Set the PID parameters
    */
    void VelocityController::setFeedforwardAndFrictionGains(const double &feedforward_gain, const double &friction_gain)
    {
        feedforward_gain_ = feedforward_gain;
        friction_gain_ = friction_gain;
    }


    double VelocityController::velocityToEffort(double joint_velocity, double command_velocity, ros::Duration& period)
    {
        last_command_velocity_ = command_velocity;
        last_joint_velocity_ = joint_velocity;
        last_period_ = period;

        double commanded_effort = 0.0;
        if(command_velocity < -1e-6 or command_velocity > 1e-6) 
        {
            double error = command_velocity - joint_velocity;
            commanded_effort = pid_controller_.computeCommand(error, period) 
                + feedforward_gain_ * command_velocity
                + friction_gain_ * sign(command_velocity);
        }

        if(commanded_effort > max_effort_)
          commanded_effort = max_effort_;

        if(commanded_effort < min_effort_)
          commanded_effort = min_effort_;

        last_effort_ = commanded_effort;
        return commanded_effort;
    }

    void VelocityController::publishState() 
    {
        if(controller_state_publisher_ && controller_state_publisher_->trylock())
        {
          controller_state_publisher_->msg_.header.stamp = ros::Time::now();
          controller_state_publisher_->msg_.set_point = last_command_velocity_;
          controller_state_publisher_->msg_.process_value = last_joint_velocity_;
          controller_state_publisher_->msg_.error = last_command_velocity_ - last_joint_velocity_;
          controller_state_publisher_->msg_.time_step = last_period_.toSec();
          controller_state_publisher_->msg_.command = last_effort_;

          double dummy;
          bool antiwindup;
          getGains(controller_state_publisher_->msg_.p,
            controller_state_publisher_->msg_.i,
            controller_state_publisher_->msg_.d,
            controller_state_publisher_->msg_.i_clamp,
            dummy,
            antiwindup);
          controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
          controller_state_publisher_->unlockAndPublish();
        }
    }

    double VelocityController::sign(const double d) 
    {
        if (d > 1e-6)
            return 1;
        else if (d < -1e-6)
            return -1;
        return 0.0;
    }
}