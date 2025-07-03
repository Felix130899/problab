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
    return data;
}
