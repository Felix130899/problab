#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "partical_filter.h"
#include "convert_sensor_data.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
        : pf_(nh)
    {
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);
        

        sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>(odom_sub_, imu_sub_, 10));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2));

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);

        //Only once for startingpose 
        pf_.init();

        publish_timer_ = nh.createTimer(ros::Duration(0.1), &FilterNode::publishParticlesCallback, this);
        ROS_INFO("Particle publishing timer started (10 Hz).");

        scan_sub_ = nh.subscribe("/scan", 1, &FilterNode::scanCallback, this);

        map_sub_ = nh.subscribe("/map", 1, &FilterNode::mapCallback, this);

        model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &FilterNode::modelStatesCallback, this);
        model_states_path_pub_ = nh.advertise<nav_msgs::Path>("/model_state_path", 10);
        model_states_path_.header.frame_id = "map";
    }

private:
    
    ros::Subscriber map_sub_;
    bool map_received_ = false;
    ros::Subscriber model_states_sub_; 
    nav_msgs::Path model_states_path_; 
    ros::Publisher model_states_path_pub_;
    
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        //ROS_INFO_STREAM("Received sensor data");
        //pf_.publishParticles();
        // std::cout << "----------------------------------------------------------------";
        SensorData data = convert_sensor_data(odom_msg, imu_msg);
        ros::Time current_time = data.timestamp;
        
        //ROS_INFO_STREAM("Sensor callback triggered at time: " << current_time.toSec() << "s");

        double dt = (last_time_.isZero()) ? 0.05 : (current_time - last_time_).toSec();
        last_time_ = current_time;
        
        double v = data.control(0); 
        double omega = data.control(1);

        pf_.motionUpdate(v, omega, dt);
        if (latest_scan_) 
        {
            pf_.updateWeights(*latest_scan_);
        }

        pf_.resample();

        pf_.estimatePose();
        
      
    }

    void publishParticlesCallback(const ros::TimerEvent& event)
    {
        pf_.publishParticles(); // Publish particles to RViz
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
    {
        latest_scan_ = msg;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
    {
    if (!map_received_) 
        {
        pf_.setMap(*msg);
        map_received_ = true;
        ROS_INFO("Static map received and set.");

        ROS_INFO_STREAM("Map dimensions: " << msg->info.width << " x " << msg->info.height);
        ROS_INFO_STREAM("Map resolution: " << msg->info.resolution << " m/cell");
        ROS_INFO_STREAM("Map origin: (" << msg->info.origin.position.x << ", "
                                        << msg->info.origin.position.y << ")");
        ROS_INFO_STREAM("Map data size: " << msg->data.size());

        map_sub_.shutdown(); 
        }
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

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu>> sync_;
    ros::Publisher pub_;
    ros::Time last_time_;

    ros::Timer publish_timer_;

    //ParticalFilter object as a member of FilterNode
    ParticalFilter pf_; 

    sensor_msgs::LaserScan::ConstPtr latest_scan_;
    ros::Subscriber scan_sub_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;
    FilterNode node(nh);
    ros::spin();

    return 0;
}
