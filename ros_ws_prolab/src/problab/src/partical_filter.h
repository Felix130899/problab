#ifndef PARTICAL_FILTER_H
#define PARTICAL_FILTER_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>

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
    // void motionUpdate();
    // void weightsUpdate();
    // void resample();
    // void estimatePose(); 

    void publishParticles();

private:
    std::vector<Particle> particales_;
    ros::Publisher particales_pub_;

};

#endif // PARTICAL_FILTER_H