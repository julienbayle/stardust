#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sd_sensor/GetBias.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

namespace sd_sensor {
class MPU6050 {
public:
    static const int I2C_ADDR = 0x68;
    static const int PWR_MGMT_1 = 0x6B;
    static const int GYRO_CONFIG = 0x1B;
    static const int ACCEL_CONFIG = 0x1C;

    static const int XG_ST_BIT = (1 << 7);
    static const int YG_ST_BIT = (1 << 6);
    static const int ZG_ST_BIT = (1 << 5);

    static const int XA_ST_BIT = (1 << 7);
    static const int YA_ST_BIT = (1 << 6);
    static const int ZA_ST_BIT = (1 << 5);

    static const int BIAS_COUNT = 100;

private:
    ros::NodeHandle nh_, nh_priv_;
    ros::Publisher imu_publisher_;
    ros::ServiceServer get_bias_service_;
    ros::Timer timer_;

    int i2c_adapter_;
    int i2c_address_;
    std::string frame_;

    int file_;

    double bias_gx_, bias_gy_, bias_gz_, bias_ax_, bias_ay_, bias_az_;

    double cp_bias_gx_, cp_bias_gy_, cp_bias_gz_, cp_bias_ax_, cp_bias_ay_, cp_bias_az_;
    int bias_count_;

    double read_word_i2c(int addr)
    {
        int high = i2c_smbus_read_byte_data(file_, addr);
        int low = i2c_smbus_read_byte_data(file_, addr + 1);
        int val = (high << 8) + low;
        return double((val >= 0x8000) ? -((65535 - val) + 1) : val);
    }

public:
    MPU6050() :
        nh_(),
        nh_priv_("~")
    {
        // Get parameters
        nh_priv_.param("i2c_adapter", i2c_adapter_, 1);
        nh_priv_.param("i2c_address", i2c_address_, 0x68);
        nh_priv_.param("frame", frame_, (std::string)"imu_link");
        nh_priv_.param("bias_gx", bias_gx_, 0.0);
        nh_priv_.param("bias_gy", bias_gy_, 0.0);
        nh_priv_.param("bias_gz", bias_gz_, 0.0);
        nh_priv_.param("bias_ax", bias_ax_, 0.0);
        nh_priv_.param("bias_ay", bias_ay_, 0.0);
        nh_priv_.param("bias_az", bias_az_, 0.0);

        // Connect to device.
        char filename[20];

        snprintf(filename, 19, "/dev/i2c-%d", i2c_adapter_);
        file_ = open(filename, O_RDWR);
        if (file_ < 0) {
            ROS_ERROR("Unable to open i2c dev %s", filename);
            exit(1);
        }

        if (ioctl(file_, I2C_SLAVE, i2c_address_) < 0) {
            ROS_ERROR("Unable to contact i2c device at address 0x%x", i2c_address_);
            exit(1);
        }

        // Device starts in sleep mode so wake it up.
        i2c_smbus_write_word_data(file_, PWR_MGMT_1, 0);

        // Configure gyroscope
        uint8_t gyro_conf = 0;//XG_ST_BIT | YG_ST_BIT | ZG_ST_BIT;
        i2c_smbus_write_byte_data(file_, GYRO_CONFIG, gyro_conf);

        // Configure gyroscope
        uint8_t accel_conf = 0;//XA_ST_BIT | YA_ST_BIT | ZA_ST_BIT;
        i2c_smbus_write_byte_data(file_, ACCEL_CONFIG, accel_conf);

        // Create publishers
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);

        // Create services
        get_bias_service_ = nh_priv_.advertiseService("get_bias", &MPU6050::get_bias_callback, this);

        // Create loop timer
        timer_ = nh_priv_.createTimer(ros::Duration(0.1), &MPU6050::timer_callback, this);
    }

    void timer_callback(const ros::TimerEvent&)
    {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_;

        // Read gyroscope values.
        // At default sensitivity of 250deg/s we need to scale by 131.
        double gx = read_word_i2c(0x43) / 131 * M_PI / 180.0;
        double gy = read_word_i2c(0x45) / 131 * M_PI / 180.0;
        double gz = -read_word_i2c(0x47) / 131 * M_PI / 180.0;

        // Read accelerometer values.
        // At default sensitivity of 2g we need to scale by 16384.
        // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
        // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
        double ax = -read_word_i2c(0x3b) / 16384.0 * 9.807;
        double ay = read_word_i2c(0x3d) / 16384.0 * 9.807;
        double az = read_word_i2c(0x3f) / 16384.0 * 9.807;

        // Read gyroscope values.
        // At default sensitivity of 250deg/s we need to scale by 131.
        msg.angular_velocity.x = gx - bias_gx_;
        msg.angular_velocity.y = gy - bias_gy_;
        msg.angular_velocity.z = gz - bias_gz_;

        const float la_rescale = 16384.0 / 9.807;
        msg.linear_acceleration.x = ax - bias_ax_;
        msg.linear_acceleration.y = ay - bias_ay_;
        msg.linear_acceleration.z = az - bias_az_;

        // Pub & sleep.
        imu_publisher_.publish(msg);

        // Compute bias
        if (bias_count_ > 0) {
            cp_bias_gx_ += gx;
            cp_bias_gy_ += gy;
            cp_bias_gz_ += gz;
            cp_bias_ax_ += ax;
            cp_bias_ay_ += ay;
            cp_bias_az_ += az + 9.807;
            bias_count_--;
        }
    }

    bool get_bias_callback(sd_sensor::GetBias::Request& request, sd_sensor::GetBias::Response& response)
    {
        cp_bias_gx_ = cp_bias_gy_ = cp_bias_gz_ = 0.0;
        cp_bias_ax_ = cp_bias_ay_ = cp_bias_az_ = 0.0;

        bias_count_ = BIAS_COUNT;
        int timeout = 15000; // 15s
        ros::Rate rate(10.0);
        
        while (bias_count_ != 0 && timeout > 0) {
            timeout -= 100; // 100ms
            rate.sleep();
        }

        if (bias_count_ != 0) {
            response.result = false;
            response.bias_gx = response.bias_gy = response.bias_gz = 0.0;
            response.bias_ax = response.bias_ay = response.bias_az = 0.0;
        } else {
            response.result = true;
            response.bias_gx = cp_bias_gx_ / BIAS_COUNT;
            response.bias_gy = cp_bias_gy_ / BIAS_COUNT;
            response.bias_gz = cp_bias_gz_ / BIAS_COUNT;
            response.bias_ax = cp_bias_ax_ / BIAS_COUNT;
            response.bias_ay = cp_bias_ay_ / BIAS_COUNT;
            response.bias_az = cp_bias_az_ / BIAS_COUNT;
        }

        return true;
    }
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu6050");
    
    sd_sensor::MPU6050 mpu6050;

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
