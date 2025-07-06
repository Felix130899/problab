#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "convert_sensor_data.h" 

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter();
    void predict(const SensorData &data, double dt);
    void correct(const SensorData &data);

    const Eigen::VectorXd& getMu() const;
    const Eigen::MatrixXd& getSigma() const;

private:
    Eigen::VectorXd mu_;     // State: [x, y, theta, v, omega]
    Eigen::VectorXd mu_p;
    Eigen::MatrixXd Sigma_;  // Covarianz
    Eigen::MatrixXd R_;      // Process noise
    Eigen::MatrixXd Q_;
};

#endif // EKF_FILTER_H