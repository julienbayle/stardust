#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sd_sensor/GetBias.h>
#include "sd_sensor/mpu6050_fifo.h"

namespace sd_sensor {

class MPU6050_FIFO {

    private:

        ros::NodeHandle nh_, nh_priv_;
        ros::Publisher imu_publisher_;
        ros::ServiceServer get_bias_service_;
        ros::Timer timer_;

        double frequency_;
        int i2c_adapter_;
        int i2c_address_;
        std::string frame_;

        float temps_;
        float accel_data_[3];
        float gyro_data_[3];

        int file_;

        double bias_gx_, bias_gy_, bias_gz_, bias_ax_, bias_ay_, bias_az_;

        double cp_bias_gx_, cp_bias_gy_, cp_bias_gz_, cp_bias_ax_, cp_bias_ay_, cp_bias_az_;
        int bias_count_;
        int bias_init_count_;
        bool update_bias_at_startup_;
        bool updating_bias_at_startup_;

        double linear_acceleration_stdev_, angular_velocity_stdev_;
        double angular_velocity_covariance_, linear_acceleration_covariance_;

        float accel_scale_factor_, gyro_scale_factor_;


    public:
        
        MPU6050_FIFO() :
            nh_(),
            nh_priv_("~")
        {
            // Get parameters
            nh_priv_.param("frequency", frequency_, 100.0);
            nh_priv_.param("i2c_adapter", i2c_adapter_, 1);
            nh_priv_.param("i2c_address", i2c_address_, 0x68);
            nh_priv_.param("frame", frame_, (std::string)"imu_link");
            nh_priv_.param("bias_gx", bias_gx_, 0.0);
            nh_priv_.param("bias_gy", bias_gy_, 0.0);
            nh_priv_.param("bias_gz", bias_gz_, 0.0);
            nh_priv_.param("bias_ax", bias_ax_, 0.0);
            nh_priv_.param("bias_ay", bias_ay_, 0.0);
            nh_priv_.param("bias_az", bias_az_, 0.0);

            // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
            nh_priv_.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807 );

            // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
            nh_priv_.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));

            angular_velocity_covariance_ = angular_velocity_stdev_ * angular_velocity_stdev_;
            linear_acceleration_covariance_ = linear_acceleration_stdev_ * linear_acceleration_stdev_;

            nh_priv_.param("update_bias_at_startup_count", bias_init_count_, 100);
            nh_priv_.param("update_bias_at_startup", update_bias_at_startup_,false);
            updating_bias_at_startup_ = update_bias_at_startup_;
            if(updating_bias_at_startup_)
            {
                bias_count_ = bias_init_count_;
                cp_bias_gx_ = cp_bias_gy_ = cp_bias_gz_ = 0.0;
                cp_bias_ax_ = cp_bias_ay_ = cp_bias_az_ = 0.0;
            }

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

            wake_up_device(file_);

            set_sample_rate(file_, frequency_);  // 100 samples per second
            enable_fifo(file_);
            reset_fifo(file_);

            accel_scale_factor_ = set_accel_scale_factor(file_, 2); // set 2G
            gyro_scale_factor_ = set_gyro_scale_factor(file_, 250); // set 250 degree

            // Create publishers
            imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);

            // Create services
            get_bias_service_ = nh_priv_.advertiseService("get_bias", &MPU6050_FIFO::get_bias_callback, this);

            ROS_INFO("Starting mpu6050_node (%1.2f Hz, frame = %s)", frequency_, frame_.c_str());
            
            // Create loop timer
            timer_ = nh_priv_.createTimer(ros::Duration(1 / frequency_), &MPU6050_FIFO::timer_callback, this);
        }

        void timer_callback(const ros::TimerEvent&)
        {
            int16_t fifo_count = get_fifo_count(file_);
            
            if (fifo_count < 14)
                return;

            if( fifo_count == 1024 || overflow_interrupt(file_) == 1)
            {
                    ROS_ERROR("IMU - Overflow Detected! Resetting FIFO buffer and counter");
                    reset_fifo(file_);
                    return;
            }
           
            read_fifo_buffer(file_, temps_, accel_data_, accel_scale_factor_, gyro_data_, gyro_scale_factor_);

            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_;

            msg.angular_velocity.x = gyro_data_[0] - bias_gx_;
            msg.angular_velocity.y = gyro_data_[1] - bias_gy_;
            msg.angular_velocity.z = gyro_data_[2] - bias_gz_;
            msg.angular_velocity_covariance[0] = angular_velocity_covariance_;
            msg.angular_velocity_covariance[4] = angular_velocity_covariance_;
            msg.angular_velocity_covariance[8] = angular_velocity_covariance_;

            msg.linear_acceleration.x = accel_data_[0] - bias_ax_;
            msg.linear_acceleration.y = accel_data_[1] - bias_ay_;
            msg.linear_acceleration.z = accel_data_[2] - bias_az_;
            msg.linear_acceleration_covariance[0] = linear_acceleration_covariance_;
            msg.linear_acceleration_covariance[4] = linear_acceleration_covariance_;
            msg.linear_acceleration_covariance[8] = linear_acceleration_covariance_;

            // Pub & sleep.
            imu_publisher_.publish(msg);

            // Compute bias
            if (bias_count_ > 0) {
                cp_bias_gx_ += gyro_data_[0];
                cp_bias_gy_ += gyro_data_[1];
                cp_bias_gz_ += gyro_data_[2];
                cp_bias_ax_ += accel_data_[0];
                cp_bias_ay_ += accel_data_[1];
                cp_bias_az_ += accel_data_[2] + 9.807;
                bias_count_--;
            }

            if(updating_bias_at_startup_ && bias_count_ == 0)
            {
                updating_bias_at_startup_ = false;
                bias_gx_ = cp_bias_gx_ / bias_init_count_;
                bias_gy_ = cp_bias_gy_ / bias_init_count_;
                bias_gz_ = cp_bias_gz_ / bias_init_count_;
                bias_ax_ = cp_bias_ax_ / bias_init_count_;
                bias_ay_ = cp_bias_ay_ / bias_init_count_;
                bias_az_ = cp_bias_az_ / bias_init_count_;

                ROS_INFO("IMU Bias updated");
            }
        }

        bool get_bias_callback(sd_sensor::GetBias::Request& request, sd_sensor::GetBias::Response& response)
        {
            cp_bias_gx_ = cp_bias_gy_ = cp_bias_gz_ = 0.0;
            cp_bias_ax_ = cp_bias_ay_ = cp_bias_az_ = 0.0;

            bias_count_ = bias_init_count_;
            int timeout = bias_init_count_ * frequency_ * 1.5;
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
                response.bias_gx = cp_bias_gx_ / bias_init_count_;
                response.bias_gy = cp_bias_gy_ / bias_init_count_;
                response.bias_gz = cp_bias_gz_ / bias_init_count_;
                response.bias_ax = cp_bias_ax_ / bias_init_count_;
                response.bias_ay = cp_bias_ay_ / bias_init_count_;
                response.bias_az = cp_bias_az_ / bias_init_count_;
            }

            return true;
        }
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu6050");
    
    sd_sensor::MPU6050_FIFO mpu6050_fifo;

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}