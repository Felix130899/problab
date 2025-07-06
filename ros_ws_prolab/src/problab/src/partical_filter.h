#ifndef PARTICAL_FILTER_H
#define PARTICAL_FILTER_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>   
#include <nav_msgs/Path.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct Particle 
{
    double x, y, theta;
    double weight;
};

const int NUM_PARTICLES = 600;

class ParticalFilter
{
public:
    ParticalFilter();
    ParticalFilter(ros::NodeHandle& nh);

    //Main function for particle filter
    void init();
    void motionUpdate(double v, double omega, double delta_t);
    void updateWeights(const sensor_msgs::LaserScan& scan);
    void resample();
    void estimatePose();

    //Helping function
    void publishParticles();
    bool worldToMap(float x, float y, int& map_x, int& map_y) const;
    float getExpectedDistanceFromMap(float x, float y, float theta);
    void setMap(const nav_msgs::OccupancyGrid& map);
    double randomNoise(double stddev);

private:
    double motion_noise_trans_;
    double motion_noise_rot_;
    double sensor_noise_;

    std::vector<Particle> particales_;
    ros::Publisher particles_marker_pub_;

    std::mt19937 gen_;
    std::normal_distribution<> noise_trans_;
    std::normal_distribution<> noise_rot_;

    nav_msgs::OccupancyGrid map_;
    std::vector<Particle> particles_;

    ros::Publisher estimated_path_pub_;
    nav_msgs::Path estimated_path_;

    geometry_msgs::PoseStamped last_smoothed_pose_;
    bool has_smoothed_pose_ = false;

};

#endif // PARTICAL_FILTER_H