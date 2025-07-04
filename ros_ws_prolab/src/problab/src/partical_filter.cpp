// partical_filter.cpp
#include "partical_filter.h"
#include <random>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ParticalFilter::ParticalFilter()
{

}

ParticalFilter::ParticalFilter(ros::NodeHandle& nh)
{
    particales_pub_= nh.advertise<sensor_msgs::PointCloud2>("particles_topic", 1);
}

void ParticalFilter::init()
{
    //Initalisiert Zufallsgenerator
    std::random_device rd;
    std::mt19937 gen(rd());

    //Standartabweichung für die Gaußche Verteilung
    double sigma_x_intit = 0.1;
    double sigma_y_intit = 0.1;
    double sigma_theta_intit = 0.05;

    //Normalverteilung um 0,0,0
    std::normal_distribution<> d_x_init(0.0, sigma_x_intit);
    std::normal_distribution<> d_y_init(0.0, sigma_y_intit);
    std::normal_distribution<> d_theta_init(0.0, sigma_theta_intit);

    //Löscht noch vorhandene Pratikel und 'particales' ist jetzt eine Membervariable der Klasse ParticalFilter
    particales_.clear();
    particales_.reserve(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle p;
        p.x = d_x_init(gen);
        p.y = d_y_init(gen);
        p.theta = d_theta_init(gen);
        p.weight = 1.0 / NUM_PARTICLES; //Anfangsgewicht gleichmäßig verteilen
        particales_.push_back(p);
    }
}


// void ParticalFilter::motionUpdate()
// {

// }

// void ParticalFilter::weightsUpdate()
// {

// }

// void ParticalFilter::resample()
// {

// }

// void ParticalFilter::estimatePose()
// {

// }

void ParticalFilter::publishParticles()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(particales_.size());

    for (size_t i = 0; i < particales_.size(); ++i) {
        cloud.points[i].x = particales_[i].x;
        cloud.points[i].y = particales_[i].y;
        cloud.points[i].z = 0.0; 
    }
    
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.stamp = ros::Time::now();

    particales_pub_.publish(output);
}

