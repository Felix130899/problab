#include "convert_sensor_data.h"

SensorData convert_sensor_data(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const sensor_msgs::Imu::ConstPtr &imu_msg) {
    SensorData data;
    data.control << odom_msg->twist.twist.linear.x,
                    odom_msg->twist.twist.angular.z;
    data.orientation = Eigen::Quaterniond(
        imu_msg->orientation.w,
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z);
    data.timestamp = odom_msg->header.stamp; // oder imu_msg->header.stamp

    double vx = odom_msg->twist.twist.linear.x;
    double vy = odom_msg->twist.twist.linear.y;
    data.linear_velocity = std::sqrt(vx*vx + vy*vy);
    
    double wz_odom = odom_msg->twist.twist.angular.z;
    data.angular_velocity_z = wz_odom;


    data.imu_msg_ptr = imu_msg;
    return data;
}
