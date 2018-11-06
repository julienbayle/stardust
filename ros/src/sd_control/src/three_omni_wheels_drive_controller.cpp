#include <cmath>

#include <tf/transform_datatypes.h>

#include <boost/assign.hpp>

#include <sd_control/three_omni_wheels_drive_controller.h>

namespace sd_control {

  ThreeOmniWheelsDriveController::ThreeOmniWheelsDriveController()
    : open_loop_(false)
    , command_struct_()
    , wheel_to_center_(0.0)
    , wheel_radius_(0.0)
    , cmd_vel_timeout_(0.5)
    , allow_multiple_cmd_vel_publishers_(true)
    , base_frame_id_("base_link")
    , odom_frame_id_("odom")
    , enable_odom_tf_(true)
    , publish_cmd_(false)
  {
  }

  bool ThreeOmniWheelsDriveController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::string left_wheel_name, right_wheel_name, back_wheel_name;
    if (!getWheelName(controller_nh, "left_wheel", left_wheel_name) or
        !getWheelName(controller_nh, "right_wheel", right_wheel_name) or
        !getWheelName(controller_nh, "back_wheel", back_wheel_name)) 
    {
      return false;
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_, allow_multiple_cmd_vel_publishers_);
    ROS_INFO_STREAM_NAMED(name_, "Allow mutiple cmd_vel publishers is "
                          << (allow_multiple_cmd_vel_publishers_?"enabled":"disabled"));

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    /*controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_x_.has_velocity_limits    , limiter_lin_x_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_x_.has_acceleration_limits, limiter_lin_x_.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits"        , limiter_lin_x_.has_jerk_limits        , limiter_lin_x_.has_jerk_limits        );
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_x_.max_velocity           ,  limiter_lin_x_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_x_.min_velocity           , -limiter_lin_x_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_x_.max_acceleration       ,  limiter_lin_x_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_x_.min_acceleration       , -limiter_lin_x_.max_acceleration      );
    controller_nh.param("linear/x/max_jerk"               , limiter_lin_x_.max_jerk               ,  limiter_lin_x_.max_jerk              );
    controller_nh.param("linear/x/min_jerk"               , limiter_lin_x_.min_jerk               , -limiter_lin_x_.max_jerk              );

    controller_nh.param("linear/y/has_velocity_limits"    , limiter_lin_y_.has_velocity_limits    , limiter_lin_y_.has_velocity_limits    );
    controller_nh.param("linear/y/has_acceleration_limits", limiter_lin_y_.has_acceleration_limits, limiter_lin_y_.has_acceleration_limits);
    controller_nh.param("linear/y/has_jerk_limits"        , limiter_lin_y_.has_jerk_limits        , limiter_lin_y_.has_jerk_limits        );
    controller_nh.param("linear/y/max_velocity"           , limiter_lin_y_.max_velocity           ,  limiter_lin_y_.max_velocity          );
    controller_nh.param("linear/y/min_velocity"           , limiter_lin_y_.min_velocity           , -limiter_lin_y_.max_velocity          );
    controller_nh.param("linear/y/max_acceleration"       , limiter_lin_y_.max_acceleration       ,  limiter_lin_y_.max_acceleration      );
    controller_nh.param("linear/y/min_acceleration"       , limiter_lin_y_.min_acceleration       , -limiter_lin_y_.max_acceleration      );
    controller_nh.param("linear/y/max_jerk"               , limiter_lin_y_.max_jerk               ,  limiter_lin_y_.max_jerk              );
    controller_nh.param("linear/y/min_jerk"               , limiter_lin_y_.min_jerk               , -limiter_lin_y_.max_jerk              );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
    controller_nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );*/

    // Publish limited velocity:
    controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

    // Mobile base parameters
    bool wheel_to_center_ok = controller_nh.getParam("wheel_to_center", wheel_to_center_);
    bool wheel_radius_ok = controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!wheel_to_center_ok or !wheel_radius_ok)
    {
      ROS_ERROR_NAMED(name_, "'wheel_to_center' or 'wheel_radius' not defined. Controller is not running.");
      return false;
    }

    odometry_.setWheelParams(wheel_to_center_, wheel_radius_);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel to center " << wheel_to_center_
                          << ", wheel radius "  << wheel_radius_);

    setOdomPubFields(root_nh, controller_nh);

    if (publish_cmd_)
    {
      cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
    }

    // Get the joint object to use in the realtime loop
    ROS_INFO_STREAM_NAMED(name_,
                            "Adding left wheel with joint name: " << left_wheel_name
                            << ", right wheel with joint name: " << right_wheel_name
                            << " and back wheel with joint name: " << back_wheel_name);
    left_wheel_joint_ = hw->getHandle(left_wheel_name);  // throws on failure
    right_wheel_joint_ = hw->getHandle(right_wheel_name);  // throws on failure
    back_wheel_joint_ = hw->getHandle(back_wheel_name);  // throws on failure

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &ThreeOmniWheelsDriveController::cmdVelCallback, this);

    return true;
  }

  void ThreeOmniWheelsDriveController::update(const ros::Time& time, const ros::Duration& period)
  {

    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin_x, last0_cmd_.lin_y, last0_cmd_.ang, time);
    }
    else
    {
      double left_pos  = left_wheel_joint_.getPosition();
      double right_pos = right_wheel_joint_.getPosition();
      double back_pos  = back_wheel_joint_.getPosition();
      
      // Estimate linear and angular velocity using joint information
      odometry_.update(left_pos, right_pos, back_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin_x = 0.0;
      curr_cmd.lin_y = 0.0;
      curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    //limiter_lin_x_.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
    //limiter_lin_y_.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y, cmd_dt);
    //limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Publish limited velocity:
    if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
    {
      cmd_vel_pub_->msg_.header.stamp = time;
      cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin_x;
      cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin_y;
      cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
      cmd_vel_pub_->unlockAndPublish();
    }

    // Compute wheels velocities:
    const double vel_left  = -1.0 * SQRT3_2 * curr_cmd.lin_x + 0.5 * curr_cmd.lin_y + wheel_to_center_ * curr_cmd.ang;
    const double vel_right = SQRT3_2 * curr_cmd.lin_x + 0.5 * curr_cmd.lin_y + wheel_to_center_ * curr_cmd.ang;
    const double vel_back = -1.0 * curr_cmd.lin_y + wheel_to_center_ * curr_cmd.ang;

    // Set wheels velocities:
    left_wheel_joint_.setCommand(vel_left);
    right_wheel_joint_.setCommand(vel_right);
    back_wheel_joint_.setCommand(vel_back);
  }

  void ThreeOmniWheelsDriveController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

  void ThreeOmniWheelsDriveController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void ThreeOmniWheelsDriveController::brake()
  {
    left_wheel_joint_.setCommand(0.0);
    right_wheel_joint_.setCommand(0.0);
    back_wheel_joint_.setCommand(0.0);
  }

  void ThreeOmniWheelsDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
      if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
      {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
            << " publishers. Only 1 publisher is allowed. Going to brake.");
        brake();
        return;
      }

      command_struct_.ang     = command.angular.z;
      command_struct_.lin_x   = command.linear.x;
      command_struct_.lin_y   = command.linear.y;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang:   " << command_struct_.ang << ", "
                             << "Lin X: " << command_struct_.lin_x << ", "
                             << "Lin Y: " << command_struct_.lin_y << ", "
                             << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool ThreeOmniWheelsDriveController::getWheelName(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::string& wheel_name)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_name = static_cast<std::string>(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }

      return true;
  }

  void ThreeOmniWheelsDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }
  

} // namespace sd_control
