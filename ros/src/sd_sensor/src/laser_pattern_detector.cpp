#include "sd_sensor_msgs/LaserPatternDetector.h"
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string.h>

namespace sd_sensor {

  class LaserPatternDetector {

    private:

    float detector_frequency_;
    ros::Time last_processed_scan_;

    std::string isTirette_pattern_;
    float isTirette_threshold_;

    std::string isLeftFree_pattern_;
    float isLeftFree_threshold_;

    std::string isRightFree_pattern_;
    float isRightFree_threshold_;

    std::string isFrontFree_pattern_;
    float isFrontFree_threshold_;
  
    std::string isBackFree_pattern_;
    float isBackFree_threshold_;

    ros::NodeHandle nh_priv_;

    ros::Subscriber laser_scan_sub_;

    ros::Publisher laser_pattern_detector_pub_;

    ros::Publisher laser_pattern_60_pub_;

  public:

    LaserPatternDetector(ros::NodeHandle& nh) :
        nh_priv_("~"),
        last_processed_scan_(ros::Time::now()),
        laser_scan_sub_(nh.subscribe("scan", 1, &LaserPatternDetector::laserScanCallback, this)),
        laser_pattern_detector_pub_(nh.advertise<sd_sensor_msgs::LaserPatternDetector>("laser_pattern_detector", 1)),
        laser_pattern_60_pub_(nh.advertise<std_msgs::String>("laser_pattern_60", 1))
    {
      nh_priv_.param<float>("detector_frequency", detector_frequency_, 5.0);
      
      nh_priv_.param<std::string>("isTirette_pattern", isTirette_pattern_, "?");
      nh_priv_.param<float>("isTirette_threshold", isTirette_threshold_, 0.1);

      nh_priv_.param<std::string>("isLeftFree_pattern", isLeftFree_pattern_, "?");
      nh_priv_.param<float>("isLeftFree_threshold", isLeftFree_threshold_, 0.1);

      nh_priv_.param<std::string>("isRightFree_pattern", isRightFree_pattern_, "?");
      nh_priv_.param<float>("isRightFree_threshold", isRightFree_threshold_, 0.1);

      nh_priv_.param<std::string>("isFrontFree_pattern", isFrontFree_pattern_, "?");
      nh_priv_.param<float>("isFrontFree_threshold", isFrontFree_threshold_, 0.1);

      nh_priv_.param<std::string>("isBackFree_pattern",isBackFree_pattern_ , "?");
      nh_priv_.param<float>("isBackFree_threshold", isBackFree_threshold_, 0.1);
    }

    void laserScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
    {
      ros::Time scan_time = scan->header.stamp;
      if (detector_frequency_ != 0.0 && scan_time - last_processed_scan_ < ros::Duration(1.0f / detector_frequency_) )
      {
          ROS_DEBUG("Ignore scan to match detector frequency (time: %f, last %f)", scan_time.toSec(), last_processed_scan_.toSec());
          return;
      }

      // Check if we want to accept this scan, if its older than 1 sec, drop it
      ros::Duration scan_age = ros::Time::now() - scan_time;
      if (scan_age > ros::Duration(1))
      {
          ROS_WARN("Scan is too old for pattern detector (%f seconds)", scan_age.toSec());
          return;
      }

      std_msgs::String pattern_msg;
      pattern_msg.data = pattern60(scan, 0.3);
      laser_pattern_60_pub_.publish(pattern_msg);

      sd_sensor_msgs::LaserPatternDetector msg;
      msg.isTirette   = detect(scan, isTirette_pattern_,   isTirette_threshold_);
      msg.isLeftFree  = detect(scan, isLeftFree_pattern_,  isLeftFree_threshold_);
      msg.isRightFree = detect(scan, isRightFree_pattern_, isRightFree_threshold_);
      msg.isFrontFree = detect(scan, isFrontFree_pattern_, isFrontFree_threshold_);
      msg.isBackFree  = detect(scan, isBackFree_pattern_,  isBackFree_threshold_);
      laser_pattern_detector_pub_.publish(msg);

      last_processed_scan_ = scan_time;
    }

    std::string pattern60(const sensor_msgs::LaserScan::ConstPtr& scan, float threshold)
    {
      int increments = scan->ranges.size() / 60;

      std::string mesure = ""; 
      float sum = 0;
      int good_increments = 0;
      for(unsigned int i = 0; i < scan->ranges.size(); i++)
      {
        if (scan->ranges[i] > scan->range_min)
        {
          sum += scan->ranges[i];
          good_increments++;
        }
        
        if((i+1) % increments == 0)
        {
          mesure += good_increments == 0 ? "-" /*std::to_string(good_increments)*/ : (sum > threshold * increments ? "H" : "L");
          sum = good_increments = 0;
        }        
      }
        
      return mesure;
    }

    bool detect(const sensor_msgs::LaserScan::ConstPtr& scan, std::string& pattern, float threshold)
    {
      int increments = 6; //scan->ranges.size() / 60;
      float sum = 0;
      int good_increments = 0;
      for(unsigned int i = 0; i < scan->ranges.size(); i++)
      {
        if (scan->ranges[i] > scan->range_min)
        {
          sum += scan->ranges[i];
          good_increments++;
        }
        if((i+1) % increments == 0)
        {
          char p = pattern[(i+1)/increments];
          float limit = threshold * good_increments;

          if (sum > limit && good_increments > 0 && p == '0')
            return false;
        
          if (sum < limit && good_increments > 0 && p == '1')
            return false;

          if (good_increments > 0 && p == 'X')
            return false;

          sum = good_increments = 0;
        }    
      }
      return true;
    }
  };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_pattern_detector");

    ros::NodeHandle nh_;

    sd_sensor::LaserPatternDetector detector_(nh_);

    ros::spin();

    return 0;
}
