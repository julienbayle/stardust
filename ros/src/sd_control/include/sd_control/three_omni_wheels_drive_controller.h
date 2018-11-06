#ifndef SD_CONTROL_DRIVE_H_
#define SD_CONTROL_DRIVE_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <sd_control/three_omni_wheels_odometry.h>
#include <diff_drive_controller/speed_limiter.h>

namespace sd_control {

  /**
   * This class makes some assumptions on the model of the robot:
   *  - wheels angles are all 120° 
   *  - the wheels are identical in radius
   *  - distance between wheel and the center of the robot are identical
   *
   * This work is derived from diff_drive_controller, 
   * my greatest thanks to the contributors of this package
   */
  class ThreeOmniWheelsDriveController
      : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    ThreeOmniWheelsDriveController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:

    // sqrt(3)/2
    static const double SQRT3_2 = 0.866025404;

    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    /// Hardware handles:
    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;
    hardware_interface::JointHandle back_wheel_joint_;

    /// Velocity command related:
    struct Commands
    {
      double lin_x;
      double lin_y;
      double ang;
      ros::Time stamp;

      Commands() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    /// Publish executed commands
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Distance between a wheel and the robot center (assuming it's the same for all wheels) :
    double wheel_to_center_;

    /// Wheel radius (assuming it's the same for all wheels):
    double wheel_radius_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to allow multiple publishers on cmd_vel topic or not:
    bool allow_multiple_cmd_vel_publishers_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    //diff_drive_controller::SpeedLimiter limiter_lin_x_;
    //diff_drive_controller::SpeedLimiter limiter_lin_y_;
    //diff_drive_controller::SpeedLimiter limiter_ang_;

    /// Publish limited velocity:
    bool publish_cmd_;

    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the wheel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelName(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::string& wheel_names);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  };

  PLUGINLIB_EXPORT_CLASS(sd_control::ThreeOmniWheelsDriveController, controller_interface::ControllerBase);
} // namespace sd_control


#endif /* SD_CONTROL_DRIVE_H_ */
