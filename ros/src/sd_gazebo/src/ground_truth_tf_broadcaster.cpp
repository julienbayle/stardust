#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

class GroundTruthTfBroadcaster {
private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber ground_truth_subscriber_;

    std::string tf_prefix_;
    std::string base_link_;
    std::string odom_link_;
public:
    GroundTruthTfBroadcaster() :
            tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle nh_priv("~");

        nh_priv.param("tf_prefix", tf_prefix_, std::string(""));

        std::ostringstream base_link_stream;
        base_link_stream << tf_prefix_ << "/base_link";
        base_link_ = base_link_stream.str();

        std::ostringstream odom_link_stream;
        odom_link_stream << tf_prefix_ << "/odom";
        odom_link_ = odom_link_stream.str();

        ground_truth_subscriber_ = nh.subscribe("ground_truth", 1, &GroundTruthTfBroadcaster::ground_truth_handler, this);
    }

    void ground_truth_handler(const nav_msgs::OdometryConstPtr& odom) {
        geometry_msgs::TransformStamped odom_to_base_link_msg;
        try{
            odom_to_base_link_msg = tf_buffer_.lookupTransform(odom_link_, base_link_, odom->header.stamp, ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        tf2::Transform odom_to_base_link_tf;
        tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

        // Create transform from odom
        tf2::Transform map_to_base_link_tf(
            tf2::Quaternion(odom->pose.pose.orientation.x, 
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w),
            tf2::Vector3(odom->pose.pose.position.x,
                odom->pose.pose.position.y,
                odom->pose.pose.position.z));

        // Compute transform
        tf2::Transform map_to_odom_tf = (map_to_base_link_tf * odom_to_base_link_tf.inverse());

        // Send transform
        geometry_msgs::TransformStamped map_to_odom_msg;
        map_to_odom_msg.header.stamp = odom->header.stamp;
        map_to_odom_msg.header.frame_id = "map";
        map_to_odom_msg.child_frame_id = odom_link_;
        map_to_odom_msg.transform = tf2::toMsg(map_to_odom_tf);
        tf_broadcaster_.sendTransform(map_to_odom_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_truth_tf_broadcaster");

    GroundTruthTfBroadcaster gttb;

    ros::spin();

    return 0;
}