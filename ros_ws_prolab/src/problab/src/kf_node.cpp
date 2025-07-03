#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "convert_sensor_data.h" // This should define SensorData
#include "kalman_filter.h"     // This should define KalmanFilter
#include <tf2/utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream> // Für Dateiausgabe
#include <iomanip> // Für Formatierung der Dateiausgabe
#include <vector>  // Für std::vector

// Structure to store data for each time step for logging purposes
struct FilterStateData {
    ros::Time timestamp;
    Eigen::Vector3d position_orientation; // x, y, theta
    Eigen::Matrix3d covariance_xy_theta;  // The 3x3 covariance sub-matrix for x, y, theta
};


class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
    {
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);

        sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>(odom_sub_, imu_sub_, 10));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2));

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/filter_path", 10);
        path_.header.frame_id = "odom";  // or "map", depending on your setup

        nh.param<std::string>("output_file", output_filename_, "filter_data.txt");

        groundtruth_path_pub_ = nh.advertise<nav_msgs::Path>("/odom_path", 10);
        groundtruth_path_.header.frame_id = "odom";

        last_time_ = ros::Time::now(); // Initialize last_time_
    }

    // Destructor to save data when the node shuts down
    ~FilterNode()
    {
        saveFilterDataToFile(output_filename_);
    }


private:
    KalmanFilter filter_; // KalmanFilter class is included from "kalman_filter.h"
    ros::Time last_time_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pub_;

    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    tf2_ros::TransformBroadcaster tf_broadcaster_; // <--- This was present to fix RViz visibility

    std::vector<FilterStateData> filter_history_;
    std::string output_filename_;

    ros::Publisher groundtruth_path_pub_;
    nav_msgs::Path groundtruth_path_;

    // Method to publish the estimated pose with covariance
    void publish_prediction(const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma, const ros::Time &stamp)
    {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = mu(0);
        msg.pose.pose.position.y = mu(1);
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, mu(2));
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Initialize the entire covariance matrix with zeros
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
        SensorData data = convert_sensor_data(odom_msg, imu_msg);
        ros::Time current_time = data.timestamp;

        double dt = (last_time_.isZero()) ? 0.05 : (current_time - last_time_).toSec();
        last_time_ = current_time;
        
        filter_.predict(data, dt);

        Eigen::VectorXd z(3);
        z << odom_msg->pose.pose.position.x,
             odom_msg->pose.pose.position.y,
             tf2::getYaw(odom_msg->pose.pose.orientation);

        // *** HIER WAR DIE KORREKTUR VORHER NUR MIT 'z' ***
        filter_.correct(z);

        // Get the filtered state and covariance after correction
        const Eigen::VectorXd& filtered_mu = filter_.getMu();
        const Eigen::MatrixXd& filtered_sigma = filter_.getSigma();

        // Store data for history/logging
        FilterStateData current_data;
        current_data.timestamp = current_time;
        current_data.position_orientation << filtered_mu(0), filtered_mu(1), filtered_mu(2);
        current_data.covariance_xy_theta = filtered_sigma.block<3,3>(0,0); 
        filter_history_.push_back(current_data);

        publish_prediction(filtered_mu, filtered_sigma, current_time);


        geometry_msgs::PoseStamped pose;
        pose.header.stamp = current_time;
        pose.header.frame_id = "odom";
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
        geometry_msgs::PoseStamped gt_pose;
        gt_pose.header.stamp = current_time;
        gt_pose.header.frame_id = "odom";
        gt_pose.pose = odom_msg->pose.pose;
        groundtruth_path_.poses.push_back(gt_pose);
        groundtruth_path_.header.stamp = current_time;
        groundtruth_path_pub_.publish(groundtruth_path_);
    }

    // Method to save collected data to a file
    void saveFilterDataToFile(const std::string& filename)
    {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "timestamp_sec,timestamp_nsec,x,y,theta,cov_xx,cov_xy,cov_xt,cov_yx,cov_yy,cov_yt,cov_tx,cov_ty,cov_tt\n";
            file << std::fixed << std::setprecision(6); 

            for (const auto& data : filter_history_) {
                file << data.timestamp.sec << ","
                     << data.timestamp.nsec << ","
                     << data.position_orientation(0) << ","
                     << data.position_orientation(1) << ","
                     << data.position_orientation(2) << ",";
                
                file << data.covariance_xy_theta(0,0) << "," << data.covariance_xy_theta(0,1) << "," << data.covariance_xy_theta(0,2) << ","
                     << data.covariance_xy_theta(1,0) << "," << data.covariance_xy_theta(1,1) << "," << data.covariance_xy_theta(1,2) << ","
                     << data.covariance_xy_theta(2,0) << "," << data.covariance_xy_theta(2,1) << "," << data.covariance_xy_theta(2,2) << "\n";
            }
            file.close();
            ROS_INFO("Filter data saved to: %s", filename.c_str());
        } else {
            ROS_ERROR("Unable to open file: %s", filename.c_str());
        }
    }

}; // End of FilterNode class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();

    return 0;
}