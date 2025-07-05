#ifndef PARTICAL_FILTER_H
#define PARTICAL_FILTER_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>




struct Particle 
{
    double x, y, theta;
    double weight;
};

const int NUM_PARTICLES = 100;



class ParticalFilter
{
public:
    ParticalFilter();
    ParticalFilter(ros::NodeHandle& nh);
    void init();
    void motionUpdate(double v, double omega, double delta_t);
    void updateWeights(const sensor_msgs::LaserScan& scan);
    // void resample();
    // void estimatePose(); 

    void publishParticles();
    bool worldToMap(float x, float y, int& map_x, int& map_y) const;
    float getExpectedDistanceFromMap(float x, float y, float theta);
    void setMap(const nav_msgs::OccupancyGrid& map);

private:
    double motion_noise_trans_;
    double motion_noise_rot_;
    double sensor_noise_;



    std::vector<Particle> particales_;
    //ros::Publisher particales_pub_;
    ros::Publisher particles_marker_pub_;

    std::mt19937 gen_; // Renamed to avoid conflict if 'gen' is used elsewhere
    std::normal_distribution<> noise_trans_;
    std::normal_distribution<> noise_rot_;

    nav_msgs::OccupancyGrid map_;
    std::vector<Particle> particles_;

};

#endif // PARTICAL_FILTER_H