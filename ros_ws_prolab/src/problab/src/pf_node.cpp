#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "partical_filter.h"

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
    {
        //###################################################################################################################//
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);

        sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>(odom_sub_, imu_sub_, 10));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2));

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
        //###################################################################################################################//
    }

private:
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        ROS_INFO_STREAM("Received sensor data");
        pf_.publishParticles();
        std::cout << "----------------------------------------------------------------";

    }

    // void publishParticlesCallback(const ros::TimerEvent& event)
    // {
    //     // This callback is triggered by the timer, ensuring a consistent publish rate
    //     pf_.publishParticles(); // Publish particles to RViz
    // }

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pub_;
    ros::Time last_time_; // You'll use this for dt calculations in your filter

    ros::Timer publish_timer_; // New: Timer for regular particle publishing

    // Your ParticalFilter object as a member of FilterNode
    ParticalFilter pf_; // <<< DECLARE IT HERE!

    // Example for A:
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 1);

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();

    return 0;
}
