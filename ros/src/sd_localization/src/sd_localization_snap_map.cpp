/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
//for point_cloud::fromROSMsg

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread/mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <sd_localization/SnapMapConfig.h>
#include <sd_localization/SnapMapDebug.h>

namespace sd_localization {
class SnapMap {
private:
    ros::Duration age_threshold_;
    double scan_rate_;

    double icp_inlier_dist_;
    double icp_inlier_threshold_;
    double icp_num_iter_;
    double angle_upper_threshold_;

    double dist_change_threshold_;
    double angle_change_threshold_;
    ros::Duration update_age_threshold_;
    double pose_covariance_trans_;

    bool debug_;

    std::string odom_frame_;
    std::string laser_frame_;
    std::string base_frame_;
    std::string map_frame_;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

    boost::shared_ptr< sensor_msgs::PointCloud2> output_cloud_;
    boost::shared_ptr< sensor_msgs::PointCloud2> scan_cloud_;

    bool we_have_a_map_;

    bool use_sim_time_;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_xyz_;
    pcl::KdTree<pcl::PointXYZ>::Ptr map_tree_;

    ros::Time last_processed_scan_;
    ros::Time last_scan_time_;
    ros::Time last_time_sent_;

    boost::mutex scan_callback_mutex_;

    ros::NodeHandle nh_, nh_priv_;

    ros::Publisher publisher_initial_pose_;
    ros::Publisher publisher_scan_points_;
    ros::Publisher publisher_scan_points_transformed_;
    ros::Publisher publisher_debug_;

    ros::Subscriber subscriber_map_;
    ros::Subscriber subscriber_laser_scan_;

    dynamic_reconfigure::Server<sd_localization::SnapMapConfig> dynamic_reconfigure_server_;
    dynamic_reconfigure::Server<sd_localization::SnapMapConfig>::CallbackType dynamic_reconfigure_callback_;

public:
    SnapMap() :
        age_threshold_(0),
        scan_rate_(2),

        icp_inlier_dist_(0.1),
        icp_inlier_threshold_(0.9),
        icp_num_iter_(250),
        angle_upper_threshold_(M_PI / 6),

        dist_change_threshold_(0.05),
        angle_change_threshold_(0.01),
        update_age_threshold_(0),
        pose_covariance_trans_(1.5),

        debug_(false),

        odom_frame_(),
        laser_frame_(),
        base_frame_(),
        map_frame_(),

        projector_(),
        tf_listener_(),
        output_cloud_(new sensor_msgs::PointCloud2()),
        scan_cloud_(new sensor_msgs::PointCloud2()),

        we_have_a_map_(false),

        use_sim_time_(false),

        cloud_xyz_(),
        map_tree_(),
        last_processed_scan_(ros::Time::now()),
        last_scan_time_(0),
        last_time_sent_(0),

        scan_callback_mutex_(),
        nh_(),
        nh_priv_("~"),

        publisher_initial_pose_(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1)),
        publisher_scan_points_(nh_priv_.advertise<sensor_msgs::PointCloud2> ("scan_points", 1)),
        publisher_scan_points_transformed_(nh_priv_.advertise<sensor_msgs::PointCloud2> ("scan_points_transformed", 1)),
        publisher_debug_(nh_priv_.advertise<sd_localization::SnapMapDebug> ("debug", 1)),

        subscriber_map_(nh_.subscribe("map", 1, &SnapMap::mapCallback, this)),
        subscriber_laser_scan_(nh_.subscribe("laser_scan", 1, &SnapMap::scanCallback, this)),

        dynamic_reconfigure_server_(),
        dynamic_reconfigure_callback_(boost::bind(&SnapMap::dynamicReconfigureCallback, this, _1, _2))
    {
        nh_priv_.param<std::string>("odom_frame", odom_frame_, "/odom");
        nh_priv_.param<std::string>("laser_frame", laser_frame_, "/laser_link");
        nh_priv_.param<std::string>("base_frame", base_frame_, "/base_link");
        nh_priv_.param<std::string>("map_frame", map_frame_, "/map");

        dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
    }
    
    void dynamicReconfigureCallback(sd_localization::SnapMapConfig &config, uint32_t level) {
        age_threshold_ = ros::Duration(config.age_threshold);
        scan_rate_ = config.scan_rate;

        icp_inlier_dist_ = config.icp_inlier_dist;
        icp_inlier_threshold_ = config.icp_inlier_threshold;
        icp_num_iter_ = config.icp_num_iter;
        angle_upper_threshold_ = config.angle_upper_threshold;
        
        dist_change_threshold_ = config.dist_change_threshold;
        angle_change_threshold_ = config.angle_change_threshold;
        update_age_threshold_ = ros::Duration(config.update_age_threshold);
        pose_covariance_trans_ = config.pose_covariance_trans;

        debug_ = config.debug;
    }

    inline void matrixAsTransfrom (const Eigen::Matrix4f &out_mat, tf::Transform& bt)
    {
        double mv[12];

        mv[0] = out_mat (0, 0) ;
        mv[4] = out_mat (0, 1);
        mv[8] = out_mat (0, 2);
        mv[1] = out_mat (1, 0) ;
        mv[5] = out_mat (1, 1);
        mv[9] = out_mat (1, 2);
        mv[2] = out_mat (2, 0) ;
        mv[6] = out_mat (2, 1);
        mv[10] = out_mat (2, 2);

        tf::Matrix3x3 basis;
        basis.setFromOpenGLSubMatrix(mv);
        tf::Vector3 origin(out_mat (0, 3),out_mat (1, 3),out_mat (2, 3));

        ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

        bt = tf::Transform(basis,origin);
    }

    void mapCallback(const nav_msgs::OccupancyGrid& msg)
    {
        ROS_DEBUG("I heard frame_id: [%s]", msg.header.frame_id.c_str());

        float resolution = msg.info.resolution;
        float width = msg.info.width;
        float height = msg.info.height;

        float posx = msg.info.origin.position.x;
        float posy = msg.info.origin.position.y;

        cloud_xyz_ = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

        //cloud_xyz_->width    = 100; // 100
        cloud_xyz_->height   = 1;
        cloud_xyz_->is_dense = false;
        std_msgs::Header header;
        header.stamp = ros::Time(0);
        header.frame_id = map_frame_;
        cloud_xyz_->header = pcl_conversions::toPCL(header);

        pcl::PointXYZ point_xyz;

        //for (unsigned int i = 0; i < cloud_xyz_->width ; i++)
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
            {
                //@TODO
                if (msg.data[x + y * width] == 100)
                {
                    point_xyz.x = (.5f + x) * resolution + posx;
                    point_xyz.y = (.5f + y) * resolution + posy;
                    point_xyz.z = 0;
                    cloud_xyz_->points.push_back(point_xyz);
                }
            }
        cloud_xyz_->width = cloud_xyz_->points.size();

        map_tree_.reset (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        map_tree_->setInputCloud (cloud_xyz_);

        pcl::toROSMsg (*cloud_xyz_, *output_cloud_);
        ROS_DEBUG("Publishing PointXYZ cloud with %ld points in frame %s", cloud_xyz_->points.size(),output_cloud_->header.frame_id.c_str());

        we_have_a_map_ = true;
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if (!we_have_a_map_)
        {
            ROS_INFO("SnapMapICP waiting for map to be published");
            return;
        }

        ros::Time scan_in_time = scan_in->header.stamp;
        ros::Time time_received = ros::Time::now();

        if (scan_rate_ != 0.0 && scan_in_time - last_processed_scan_ < ros::Duration(1.0f / scan_rate_) )
        {
            ROS_DEBUG("rejected scan, last %f , this %f", last_processed_scan_.toSec() ,scan_in_time.toSec());
            return;
        }

        //projector_.transformLaserScanToPointCloud(base_frame_,*scan_in,cloud,tf_listener_);
        boost::mutex::scoped_lock scoped_lock(scan_callback_mutex_);

        ros::Duration scan_age = ros::Time::now() - scan_in_time;

        //check if we want to accept this scan, if its older than 1 sec, drop it
        if (!age_threshold_.isZero() && scan_age > age_threshold_)
        {
            ROS_WARN("SCAN SEEMS TOO OLD (%f seconds, %f threshold) scan time: %f , now %f", scan_age.toSec(), age_threshold_.toSec(), scan_in_time.toSec(),ros::Time::now().toSec() );

            return;
        }

        if (!tf_listener_.waitForTransform(odom_frame_, base_frame_, scan_in_time, ros::Duration(0.05)))
        {
            ROS_WARN("SnapMapICP no odom to base transform found");
            return;
        }

        tf::StampedTransform base_at_laser;
        tf_listener_.lookupTransform(odom_frame_, base_frame_, scan_in_time, base_at_laser);

        sensor_msgs::PointCloud cloud;
        sensor_msgs::PointCloud cloudInMap;

        projector_.projectLaser(*scan_in,cloud);

        if (!tf_listener_.waitForTransform(map_frame_, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.05)))
        {
            ROS_WARN("SnapMapICP no map to cloud transform found");
            return;
        }

        if (!tf_listener_.waitForTransform(map_frame_, base_frame_, cloud.header.stamp, ros::Duration(0.05)))
        {
            ROS_WARN("SnapMapICP no map to base transform found");
            return;
        }

        tf_listener_.transformPointCloud (map_frame_, cloud, cloudInMap);

        // Set zero altitude
        for (size_t k =0; k < cloudInMap.points.size(); k++)
        {
            cloudInMap.points[k].z = 0;
        }
  
        tf::StampedTransform oldPose;
        tf_listener_.lookupTransform(map_frame_, base_frame_, cloud.header.stamp , oldPose);

        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::convertPointCloudToPointCloud2(cloudInMap,cloud2);

        last_scan_time_ = ros::Time::now();

        //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setTransformationEpsilon (1e-6);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(0.5);
        reg.setMaximumIterations (icp_num_iter_);
        // Set the point representation

        PointCloudT::Ptr myMapCloud (new PointCloudT());
        PointCloudT::Ptr myScanCloud (new PointCloudT());

        pcl::fromROSMsg(*output_cloud_,*myMapCloud);
        pcl::fromROSMsg(cloud2,*myScanCloud);

        reg.setInputSource(myScanCloud);
        reg.setInputTarget(myMapCloud);

        PointCloudT unused;
        int i = 0;

        reg.align (unused);

        const Eigen::Matrix4f &transf = reg.getFinalTransformation();
        tf::Transform t;
        matrixAsTransfrom(transf,t);

        PointCloudT transformedCloud;
        pcl::transformPointCloud (*myScanCloud, transformedCloud, reg.getFinalTransformation());

        double inlier_perc = 0;
        {
            // count inliers
            std::vector<int> nn_indices (1);
            std::vector<float> nn_sqr_dists (1);

            size_t numinliers = 0;

            for (size_t k = 0; k < transformedCloud.points.size(); ++k )
            {
                if (map_tree_->radiusSearch (transformedCloud.points[k], icp_inlier_dist_, nn_indices,nn_sqr_dists, 1) != 0)
                    numinliers += 1;
            }
            if (transformedCloud.points.size() > 0)
            {
                //ROS_INFO("Inliers in dist %f: %zu of %zu percentage %f (%f)", icp_inlier_dist_, numinliers, transformedCloud.points.size(), (double) numinliers / (double) transformedCloud.points.size(), icp_inlier_threshold_);
                inlier_perc = (double) numinliers / (double) transformedCloud.points.size();
            }
        }

        last_processed_scan_ = scan_in_time;

        double dist = sqrt((t.getOrigin().x() * t.getOrigin().x()) + (t.getOrigin().y() * t.getOrigin().y()));
        double angleDist = t.getRotation().getAngle();
        tf::Vector3 rotAxis  = t.getRotation().getAxis();
        t =  t * oldPose;

        tf::StampedTransform base_after_icp;
        tf_listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), base_after_icp);

        tf::Transform rel = base_at_laser.inverseTimes(base_after_icp);
        ROS_DEBUG("relative motion of robot while doing icp: %fcm %fdeg", rel.getOrigin().length(), rel.getRotation().getAngle() * 180 / M_PI);
        t= t * rel;


        double cov = pose_covariance_trans_;
        bool sent = false;

        if ((update_age_threshold_.isZero() || (last_scan_time_ - last_time_sent_ > update_age_threshold_)) && ((dist > dist_change_threshold_) || (angleDist > angle_change_threshold_)) && (inlier_perc > icp_inlier_threshold_) && (angleDist < angle_upper_threshold_))
        {
            last_time_sent_ = last_scan_time_;
            geometry_msgs::PoseWithCovarianceStamped pose;
            pose.header.frame_id = map_frame_;
            pose.pose.pose.position.x = t.getOrigin().x();
            pose.pose.pose.position.y = t.getOrigin().y();

            tf::Quaternion quat = t.getRotation();
            tf::quaternionTFToMsg(quat,pose.pose.pose.orientation);
            float factorPos = 0.03;
            float factorRot = 0.1;
            pose.pose.covariance[6*0+0] = (cov * cov) * factorPos;
            pose.pose.covariance[6*1+1] = (cov * cov) * factorPos;
            pose.pose.covariance[6*3+3] = (M_PI/12.0 * M_PI/12.0) * factorRot;
            publisher_initial_pose_.publish(pose);

            sent = true;
        }

        if (debug_) {
            publisher_scan_points_.publish(cloud2);

            sensor_msgs::PointCloud2 cloud2_transformed;
            pcl::toROSMsg (transformedCloud, cloud2_transformed);
            publisher_scan_points_transformed_.publish(cloud2_transformed);

            // Publish debug info
            sd_localization::SnapMapDebug debug;
            debug.inlier_percentage = inlier_perc;
            debug.inlier_percentage_threshold = icp_inlier_threshold_;
            debug.scan_age = scan_age.toSec();
            debug.scan_age_threshold = age_threshold_.toSec();
            debug.distance = dist;
            debug.angle = angleDist;
            debug.rotation_axis.x = rotAxis.x();
            debug.rotation_axis.y = rotAxis.y();
            debug.rotation_axis.z = rotAxis.z();
            debug.regression_converged = reg.hasConverged();
            debug.fitness_score = reg.getFitnessScore();
            debug.sent = sent;
            publisher_debug_.publish(debug);
        }
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sd_localization_snap_map");

    sd_localization::SnapMap snap_map_;

    ros::spin();

    return 0;
}
