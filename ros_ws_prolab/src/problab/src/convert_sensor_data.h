#pragma once
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

struct SensorData {
    Eigen::Vector2d control;     // v, omega
    Eigen::Quaterniond orientation;
    ros::Time timestamp;
};

SensorData convert_sensor_data(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const sensor_msgs::Imu::ConstPtr &imu_msg);
