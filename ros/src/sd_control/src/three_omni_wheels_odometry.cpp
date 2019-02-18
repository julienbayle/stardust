#include <sd_control/three_omni_wheels_odometry.h>

#include <boost/bind.hpp>

namespace sd_control
{
  namespace bacc = boost::accumulators;

  const double Odometry::SQRT3_3 = 0.577350269;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , wheel_to_center_(0.0)
  , wheel_radius_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , back_wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_x_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , linear_y_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateRungeKutta2, this, _1, _2, _3)) {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double left_pos, double right_pos, double back_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * wheel_radius_;
    const double right_wheel_cur_pos = right_pos * wheel_radius_;
    const double back_wheel_cur_pos  = back_pos  * wheel_radius_;


    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;
    const double back_wheel_est_vel  = back_wheel_cur_pos  - back_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;
    back_wheel_old_pos_  = back_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear_x  = SQRT3_3 * (right_wheel_est_vel - left_wheel_est_vel);
    const double linear_y  =  ( -2.0 * back_wheel_est_vel + right_wheel_est_vel + left_wheel_est_vel) / 3.0;
    const double angular = ( back_wheel_est_vel + right_wheel_est_vel + left_wheel_est_vel) / (3.0 * wheel_to_center_);

    /// Integrate odometry:
    integrate_fun_(linear_x, linear_y, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_x_acc_(linear_x / dt);
    linear_y_acc_(linear_y / dt);
    angular_acc_(angular / dt);

    linear_x_ = bacc::rolling_mean(linear_x_acc_);
    linear_y_ = bacc::rolling_mean(linear_y_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_x_ = linear_x;
    linear_y_ = linear_y;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear_x * dt, linear_y * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheel_to_center, double wheel_radius)
  {
    wheel_to_center_ = wheel_to_center;
    wheel_radius_    = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
  {
    const double cos_direction = cos(heading_ + angular * 0.5);
    const double sin_direction = sin(heading_ + angular * 0.5);
    
    /// Runge-Kutta 2nd order integration:
    x_       += cos_direction * linear_x - sin_direction * linear_y;
    y_       += sin_direction * linear_x + cos_direction * linear_y;
    heading_ += angular;
  }

  void Odometry::resetAccumulators()
  {
    linear_x_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_y_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace sd_control
