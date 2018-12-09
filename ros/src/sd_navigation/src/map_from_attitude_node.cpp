#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>

#include <dynamic_reconfigure/server.h>
#include <sd_navigation/MapFromAttitudeConfig.h>

namespace sd_navigation {
class MapFromAttitude {
private:
    ros::NodeHandle nh_;

    double attitude_treshold_;
    bool use_map_2_;

    nav_msgs::OccupancyGrid map_1_, map_2_;

    ros::Publisher map_publisher_;
    ros::Subscriber map_1_subscriber_, map_2_subscriber_, imu_subscriber_;

    dynamic_reconfigure::Server<sd_navigation::MapFromAttitudeConfig> dynamic_reconfigure_server_;
    dynamic_reconfigure::Server<sd_navigation::MapFromAttitudeConfig>::CallbackType dynamic_reconfigure_callback_;

public:
    MapFromAttitude() :
        nh_(),
        attitude_treshold_(1.0),
        use_map_2_(true),
        
        map_publisher_(nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true)),
        map_1_subscriber_(nh_.subscribe("map_1", 1, &MapFromAttitude::map1Handler, this)),
        map_2_subscriber_(nh_.subscribe("map_2", 1, &MapFromAttitude::map2Handler, this)),
        imu_subscriber_(nh_.subscribe("imu/data", 1, &MapFromAttitude::imuHandler, this)),

        dynamic_reconfigure_server_(),
        dynamic_reconfigure_callback_(boost::bind(&MapFromAttitude::dynamicReconfigureCallback, this, _1, _2))
    {
        dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
    }

    void map1Handler(const nav_msgs::OccupancyGridConstPtr& map_1) {
        map_1_ = *map_1;
    }

    void map2Handler(const nav_msgs::OccupancyGridConstPtr& map_2) {
        map_2_ = *map_2;
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& imu) {
        double attitude = sqrt(pow(imu->orientation.x, 2) + pow(imu->orientation.y, 2));

        if (attitude > attitude_treshold_) {
            if (!use_map_2_) {
                map_publisher_.publish(map_2_);
                use_map_2_ = true;
            }
        } else {
            if (use_map_2_) {
                map_publisher_.publish(map_1_);
                use_map_2_ = false;
            }
        }
    }

    void dynamicReconfigureCallback(sd_navigation::MapFromAttitudeConfig &config, uint32_t level) {
        attitude_treshold_ = config.attitude_treshold;
    }

    virtual ~MapFromAttitude() {}
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_from_attitude_node");

    sd_navigation::MapFromAttitude mapFromAttitude;

    ros::spin();

    return 0;
}