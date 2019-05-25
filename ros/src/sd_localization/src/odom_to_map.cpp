/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Ioan Sucan */
/** \modified by Julien BAYLE */

#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include "message_filters/subscriber.h"

class OdomToMap
{

  private:
  
    tf2_ros::TransformBroadcaster       *m_tfServer;
    tf2_ros::TransformListener          *m_tfListener;
    tf2_ros::Buffer                     *m_tfBuffer;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;
    double transform_tolerance_;

    tf2::Transform m_offsetTf;

    tf2_ros::MessageFilter<nav_msgs::Odometry>* filter_;
    ros::Subscriber stuff_sub_; 
    message_filters::Subscriber<nav_msgs::Odometry>* filter_sub_;

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;
    tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;

    ros::Publisher m_posePub;
    
  public:
    OdomToMap(void)
    {
        // Listen to TF    
        m_tfServer = new tf2_ros::TransformBroadcaster();
        m_tfBuffer = new tf2_ros::Buffer();
        m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

        // Read parameters
        ros::NodeHandle private_nh("~");
        private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
        private_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
        private_nh.param("global_frame_id", global_frame_id_, std::string("/map"));
        private_nh.param("transform_tolerance", transform_tolerance_, 0.1);      
        
        // Init odom to map transform to "the same position"
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        m_offsetTf = tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0));

        // Subscribe to odom topic
        ros::NodeHandle nh;
        stuff_sub_ = nh.subscribe("odom", 100, &OdomToMap::waitTFFilter, this);
        filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "", 100);
        filter_ = new tf2_ros::MessageFilter<nav_msgs::Odometry>(*filter_sub_, *m_tfBuffer, base_frame_id_, 100, nh);
        filter_->registerCallback(boost::bind(&OdomToMap::update, this, _1));
        
        // Subscribe to "2D Pose Estimate"
        m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, "initialpose", 1);
        m_initPoseFilter = new tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, *m_tfBuffer, global_frame_id_, 1, nh);
        m_initPoseFilter->registerCallback(boost::bind(&OdomToMap::mapPoseReceived, this, _1));
    
        // Publish pose like AMCL
        m_posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,true);
    }

    ~OdomToMap(void)
    {
      // Clear TF buffers and listeners
      if (m_tfServer)
        delete m_tfServer; 
      if (m_tfListener)
        delete m_tfListener;
      if (m_tfBuffer)
        delete m_tfBuffer;
    }

    void waitTFFilter(const nav_msgs::OdometryConstPtr& odom_msg){
      //we have to do this to force the message filter to wait for transforms
      //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp
      //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense
      boost::shared_ptr<nav_msgs::Odometry> stuff_msg(new nav_msgs::Odometry);
      *stuff_msg = *odom_msg;
      stuff_msg->header.frame_id = odom_frame_id_;
      filter_->add(stuff_msg);
}

    void update(const nav_msgs::OdometryConstPtr& message){
      // txi :  odom_frame_id_ to base_frame_id_
      tf2::Transform txi;
      tf2::convert(message->pose.pose, txi);

      // txi : map_frame_id_ to base_frame_id_
      txi = m_offsetTf * txi;

      geometry_msgs::TransformStamped odom_to_map;
      try
      {
        // txi_inv : base_frame_id_ to map_frame_id_
        geometry_msgs::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = message->header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);
        
        m_tfBuffer->transform(txi_inv, odom_to_map, odom_frame_id_);
      }
      catch(tf2::TransformException &e)
      {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        return;
      }

      // trans : odom_frame_id_ to map_frame_id
      geometry_msgs::TransformStamped trans;
      trans.header.stamp = message->header.stamp + ros::Duration(transform_tolerance_);
      trans.header.frame_id = global_frame_id_;
      trans.child_frame_id = message->header.frame_id;
      tf2::Transform odom_to_map_tf2;
      tf2::convert(odom_to_map.transform, odom_to_map_tf2);
      tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
      tf2::convert(odom_to_map_inv, trans.transform);
      m_tfServer->sendTransform(trans);

      tf2::Transform current;
      tf2::convert(message->pose.pose, current);

      //also apply the offset to the pose
      current = m_offsetTf * current;

      geometry_msgs::Transform current_msg;
      tf2::convert(current, current_msg);

      // Publish localized pose
      geometry_msgs::PoseWithCovarianceStamped m_currentPos;
      m_currentPos.header = message->header;
      m_currentPos.header.frame_id = global_frame_id_;
      tf2::convert(current_msg.rotation, m_currentPos.pose.pose.orientation);
      m_currentPos.pose.pose.position.x = current_msg.translation.x;
      m_currentPos.pose.pose.position.y = current_msg.translation.y;
      m_currentPos.pose.pose.position.z = current_msg.translation.z;
      m_posePub.publish(m_currentPos);
    }

    void mapPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
      tf2::Transform pose;
      tf2::convert(msg->pose.pose, pose);

      if (msg->header.frame_id != global_frame_id_){
        ROS_WARN("Frame ID of \"initialpose\" (%s) is different from the global frame %s", msg->header.frame_id.c_str(), global_frame_id_.c_str());
      }

      // set offset so that current pose is set to "initialpose"    
      geometry_msgs::TransformStamped baseInMap;
      try{
        baseInMap = m_tfBuffer->lookupTransform(base_frame_id_, global_frame_id_, ros::Time(0));
      } catch(tf2::TransformException){
        ROS_WARN("Failed to lookup transform!");
        return;
      }

      tf2::Transform baseInMapTf2;
      tf2::convert(baseInMap.transform, baseInMapTf2);
      tf2::Transform delta = pose * baseInMapTf2;
      m_offsetTf = delta * m_offsetTf;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OdomToMap");

  OdomToMap odom;

  ros::spin();

  return 0;
}
