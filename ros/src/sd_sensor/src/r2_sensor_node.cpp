#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

ros::Publisher left_point_cloud_publisher;
ros::Publisher right_point_cloud_publisher;

ros::Subscriber left_range_subscriber;
ros::Subscriber right_range_subscriber;

void range_to_point_cloud(const sensor_msgs::RangeConstPtr &range, sensor_msgs::PointCloud& point_cloud) {
	point_cloud.header = range->header;

	geometry_msgs::Point32 point;
	point.x = range->range;
	point.y = 0;
	point.z = 0;

	point_cloud.points.push_back(point);
}

void left_range_handler(const sensor_msgs::RangeConstPtr &data) {
	sensor_msgs::PointCloud point_cloud;

	range_to_point_cloud(data, point_cloud);

	left_point_cloud_publisher.publish(point_cloud);
}

void right_range_handler(const sensor_msgs::RangeConstPtr &data) {
	sensor_msgs::PointCloud point_cloud;

	range_to_point_cloud(data, point_cloud);

	right_point_cloud_publisher.publish(point_cloud);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "r2_sensor_node");

	ros::NodeHandle nh;

	// Create publishers
	left_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud>("sonar/left_point_cloud", 1, false);
	right_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud>("sonar/right_point_cloud", 1, false);

	// Create subscribers
	left_range_subscriber = nh.subscribe<sensor_msgs::Range>("sonar/left", 1, left_range_handler);
	right_range_subscriber = nh.subscribe<sensor_msgs::Range>("sonar/right", 1, right_range_handler);

	ros::spin();

	return 0;
}
