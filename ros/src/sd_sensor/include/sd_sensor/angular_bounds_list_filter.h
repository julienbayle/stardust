/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Yann Bourrigault
*********************************************************************/
#ifndef LASER_SCAN_ANGULAR_BOUNDS_REPEAT_FILTER_H
#define LASER_SCAN_ANGULAR_BOUNDS_REPEAT_FILTER_H

#include <ros/ros.h>
#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

namespace sd_sensor
{
  class LaserScanAngularBoundsListFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:
      XmlRpc::XmlRpcValue angles_, angles2_;
      ros::Subscriber angles2_subscriber_;

      bool use_angles2_;

      bool configure()
      {
        XmlRpc::XmlRpcValue values;
        if(!getParam("angles", angles_)){
          ROS_ERROR("'angles' parameter must be set to use this filter.");
          return false;
        }
        if(!getParam("angles2", angles2_)){
          ROS_ERROR("'angles2' parameter must be set to use this filter.");
          return false;
        }

        ros::NodeHandle nh_priv("~");
        angles2_subscriber_ = nh_priv.subscribe("use_angles2", 1, &LaserScanAngularBoundsListFilter::use_angles2_handler, this);

        return true;
      }

      LaserScanAngularBoundsListFilter() :
        use_angles2_(false)
      {

      }

      virtual ~LaserScanAngularBoundsListFilter(){}

      void use_angles2_handler(const std_msgs::BoolConstPtr& data) {
        use_angles2_ = data->data;
      }

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
        filtered_scan = input_scan; //copy entire message
        XmlRpc::XmlRpcValue& angles = use_angles2_ ? angles2_ : angles_;

        unsigned int current_list_index = 0;
        XmlRpc::XmlRpcValue current_list_value = angles[current_list_index];
        double current_angle = input_scan.angle_min;
        unsigned int count = 0;
        //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i) {
          // Switch to next range
          if (current_angle > ((double)current_list_value[1]) && current_list_index < angles.size() - 1) {
            current_list_index++;
            current_list_value = angles[current_list_index];
          }

          // Remove scans
          if (current_angle < ((double)current_list_value[0]) || current_angle > ((double)current_list_value[1])) {
            filtered_scan.ranges[i] = input_scan.range_max + 1.0;
            if(i < filtered_scan.intensities.size()){
              filtered_scan.intensities[i] = 0.0;
            }
            count++;
          }
          current_angle += input_scan.angle_increment;
        }

        ROS_DEBUG("Filtered out %d points from the laser scan.", (int)count);

        return true;

      }
  };
};
#endif
