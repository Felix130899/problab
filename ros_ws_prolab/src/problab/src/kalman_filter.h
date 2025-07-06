#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <iomanip>
#include "convert_sensor_data.h" 


class KalmanFilter
{
public:
    KalmanFilter();
    void predict(const SensorData &data, double dt);
    void correct(const SensorData &data); 

    const Eigen::VectorXd& getMu() const;
    const Eigen::MatrixXd& getSigma() const;

private:
    Eigen::VectorXd mu_;     // Sate: x, y, theta, v, omega
    Eigen::MatrixXd Sigma_;  // Covariancematrix
    Eigen::MatrixXd R_;      // Process Noise
};

#endif // KALMAN_FILTER_H