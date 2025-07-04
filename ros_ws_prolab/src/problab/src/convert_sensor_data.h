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

    // double odom_pose_x;
    // double odom_pose_y;

    double imu_angular_velocity_z;   // aus IMU
    double odom_angular_velocity_z;  // aus Odom
    double odom_linear_velocity;     // aus Odom (x-Richtung)

};

SensorData convert_sensor_data(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const sensor_msgs::Imu::ConstPtr &imu_msg);

#endif