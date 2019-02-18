#ifndef SD_HARDWARE_INTERFACE_ENCODER_H
#define SD_HARDWARE_INTERFACE_ENCODER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace sd_hardware_interface
{

  namespace bacc = boost::accumulators;

  /// \brief Encoder
  class Encoder
  {
  public:

    /** \brief Constructor */
    Encoder(
        double encoder_steps_for_one_joint_revolution,
        double timeout,
        double max_position,
        double min_position);

    /** \brief Update encoder position callbacks */
    virtual void update(double encoder_position);
    virtual void updateFromInt16Topic(const std_msgs::Int16ConstPtr &encoder_position);

    /** \brief Return current joint angle (rad) */
    double getAngle();

    /** \brief Return current joint speed (rad/s) */
    double getSpeed();

    /** \brief Set current joint direction (for simple encoders without quadrature) */
    void setDirection(bool direction);

    /** \brief Returns if current joint encoder emits data */
    bool isAlive();

  private:

    // Used to convert seconds elapsed to nanoseconds
    static const double BILLION;

    // Max encoder value
    double max_position_;

    // Min encoder value
    double min_position_;
    
    // Last encoder position (in steps)
    double last_encoder_position_;

    // Encoder resolution (rad/step)
    double encoder_resolution_;

    // Current joint angle
    double angle_;

    // Current joint speed
    double speed_;

    // Current joint rotation direction (if encoder type is not a quadrature one)
    double direction_;

    // Current joint speed (rolling mean)
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;
    RollingMeanAcc speed_acc_;

    // Timestamp for speed and timout computation
    ros::Duration elapsed_time_;
    struct timespec last_time_;
    struct timespec current_time_;

    // Encoder timeout in seconds
    double timeout_;
  };

}	// sd_hardware_interface

#endif	// SD_HARDWARE_INTERFACE_MOTOR_ENCODER_H