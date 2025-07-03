// convert_sensor_data.h
#ifndef CONVERT_SENSOR_DATA_H
#define CONVERT_SENSOR_DATA_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> // Ensure Eigen is included

struct SensorData {
    ros::Time timestamp;
    Eigen::Vector2d control; // <--- Changed to fixed size 2D vector
    Eigen::Quaterniond orientation;
    // ... other members if any
    sensor_msgs::Imu::ConstPtr imu_msg_ptr; 
    double linear_velocity = 0.0;
    double angular_velocity_z = 0.0;

};

SensorData convert_sensor_data(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const sensor_msgs::Imu::ConstPtr &imu_msg);

#endif