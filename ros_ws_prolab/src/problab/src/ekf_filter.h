#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
// Make sure kalman_filter.h includes convert_sensor_data.h
// so it knows about the SensorData struct.
#include "convert_sensor_data.h" 

// REMOVE THE SensorData struct definition from here!

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter();
    void predict(const SensorData &data, double dt);
    void correct(const SensorData &data);

    const Eigen::VectorXd& getMu() const;
    const Eigen::MatrixXd& getSigma() const;

private:
    Eigen::VectorXd mu_;     // Zustand: [x, y, theta, v, omega]
    Eigen::MatrixXd Sigma_;  // Kovarianzmatrix
    Eigen::MatrixXd R_;      // Prozessrauschen
    Eigen::MatrixXd Q_;
};

#endif // EKF_FILTER_H