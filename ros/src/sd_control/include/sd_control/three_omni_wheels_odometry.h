#ifndef SD_CONTROL_ODOMETRY_H_
#define SD_CONTROL_ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace sd_control
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry calculus
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    // sqrt(3)/3
    static const double SQRT3_3;

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param left_pos  Left  wheel position [rad]
     * \param right_pos Right wheel position [rad]
     * \param back_pos  Back  wheel position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(double left_pos, double right_pos, double back_pos, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear_x  Linear velocity on X axis [m/s]
     * \param linear_y  Linear velocity on Y axis [m/s]
     * \param angular   Angular velocity [rad/s]
     * \param time      Current time
     */
    void updateOpenLoop(double linear_x, double linear_y, double angular, const ros::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter (X axis)
     * \return linear velocity [m/s]
     */
    double getLinearX() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter (Y axis)
     * \return linear velocity [m/s]
     */
    double getLinearY() const
    {
      return linear_y_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief Sets the wheel parameters: radius and distance to center
     * \param wheel_to_center    Distance from a wheel to the robot center [m]
     * \param wheel_radius       Xheel radius [m]
     */
    void setWheelParams(double wheel_to_center, double wheel_radius);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear_x, double linear_y, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_x_;  //   [m/s]
    double linear_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheel_to_center_;
    double wheel_radius_;

    /// Previous wheel position/state [rad]:
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;
    double back_wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_x_acc_;
    RollingMeanAcc linear_y_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
}

#endif /* SD_CONTROL_ODOMETRY_H_ */
