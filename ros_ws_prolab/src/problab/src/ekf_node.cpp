#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "convert_sensor_data.h"
#include "ekf_filter.h"   
#include <tf2/utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <iomanip>
#include <vector>
#include <gazebo_msgs/ModelStates.h>

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
    {
        //Subscribes to the Sensors of the turtlebot
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);

        sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>(odom_sub_, imu_sub_, 10));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2));
        
        //Publish the EKF Prediction in realtime and over time as a path
        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/ekf_path", 10);
        path_.header.frame_id = "map";

        //Initsialisiert last_time_
        last_time_ = ros::Time::now();



        model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &FilterNode::modelStatesCallback, this);
        model_states_path_pub_ = nh.advertise<nav_msgs::Path>("/model_state_path", 10);
        model_states_path_.header.frame_id = "map";

    }
    ~FilterNode()
    {

    }


private:

    ExtendedKalmanFilter filter_;
    ros::Time last_time_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    
    //For EKF prediction over time and realtime
    ros::Publisher pub_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    ros::Subscriber model_states_sub_; 
    nav_msgs::Path model_states_path_;
    ros::Publisher model_states_path_pub_;

    //Method to publish the estimated pose with covariance
    void publish_prediction(const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma, const ros::Time &stamp)
    {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        //Poses from the Predicition mu
        msg.pose.pose.position.x = mu(0);
        msg.pose.pose.position.y = mu(1);
        msg.pose.pose.position.z = 0.0; //always Zero because Bot moves only in x,y

        //Convert to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, mu(2));
        msg.pose.pose.orientation = tf2::toMsg(q);

        //Initialize the entire covariance matrix with zeros
        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = 0.0;
        }

        // Copy the 3x3 covariance sub-matrix for x, y, theta (indices 0, 1, 2 in our 5x5 Sigma)
        // Mapping from our Eigen::Matrix (x, y, theta) to ROS 6x6 (x, y, z, roll, pitch, yaw)
        // Indices in ROS 6x6 array for x, y, yaw are 0, 1, 5 respectively.
        msg.pose.covariance[0] = Sigma(0,0);   // x-x
        msg.pose.covariance[1] = Sigma(0,1);   // x-y
        msg.pose.covariance[5] = Sigma(0,2);   // x-theta (yaw)
        
        msg.pose.covariance[6] = Sigma(1,0);   // y-x
        msg.pose.covariance[7] = Sigma(1,1);   // y-y
        msg.pose.covariance[11] = Sigma(1,2);  // y-theta (yaw)

        msg.pose.covariance[30] = Sigma(2,0);  // theta-x (yaw-x)
        msg.pose.covariance[31] = Sigma(2,1);  // theta-y (yaw-y)
        msg.pose.covariance[35] = Sigma(2,2);  // theta-theta (yaw-yaw)
        
        pub_.publish(msg);
    }

    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        //Calls Function to convert the sensor data for easy access
        SensorData data = convert_sensor_data(odom_msg, imu_msg);
        ros::Time current_time = data.timestamp;
        
        //Calculats dt 
        double dt = (last_time_.isZero()) ? 0.05 : (current_time - last_time_).toSec();
        last_time_ = current_time;
        
        //Calls the predict function for KF, with data from the convert function and dt
        filter_.predict(data, dt);

        //Calls the correct function for KF, with data from teh convert function
        filter_.correct(data); 

        // Get the filtered state and covariance after correction
        const Eigen::VectorXd& filtered_mu = filter_.getMu();
        const Eigen::MatrixXd& filtered_sigma = filter_.getSigma();

        //Publishes the prediction after the correction
        publish_prediction(filtered_mu, filtered_sigma, current_time);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = current_time;
        pose.header.frame_id = "map";
        pose.pose.position.x = filtered_mu(0);
        pose.pose.position.y = filtered_mu(1);
        pose.pose.position.z = 0.0;

        tf2::Quaternion q_path;
        q_path.setRPY(0, 0, filtered_mu(2));
        pose.pose.orientation = tf2::toMsg(q_path);

        path_.poses.push_back(pose);
        path_.header.stamp = current_time;
        path_pub_.publish(path_);

        // Publish "Ground Truth" path from odometry for comparison
        // geometry_msgs::PoseStamped gt_pose;
        // gt_pose.header.stamp = current_time;
        // gt_pose.header.frame_id = "odom";
        // gt_pose.pose = odom_msg->pose.pose;
        // groundtruth_path_.poses.push_back(gt_pose);
        // groundtruth_path_.header.stamp = current_time;
        // groundtruth_path_pub_.publish(groundtruth_path_);

        //ROS_INFO_STREAM("Finished sensor callback loop.");
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        auto it = std::find(msg->name.begin(), msg->name.end(), "turtlebot3");
        if (it == msg->name.end()) {
            ROS_WARN_THROTTLE(5.0, "Robot name not found in model_states");
            return;
        }

        size_t index = std::distance(msg->name.begin(), it);
        geometry_msgs::PoseStamped gt_pose;
        gt_pose.header.stamp = ros::Time::now();
        gt_pose.header.frame_id = "map";
        gt_pose.pose = msg->pose[index];

        model_states_path_.poses.push_back(gt_pose);
        model_states_path_.header.stamp = gt_pose.header.stamp;
        model_states_path_pub_.publish(model_states_path_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();

    return 0;
}