#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
// Make sure kalman_filter.h includes convert_sensor_data.h
// so it knows about the SensorData struct.
#include "convert_sensor_data.h" 

// REMOVE THE SensorData struct definition from here!
// It should ONLY be in convert_sensor_data.h

class KalmanFilter
{
public:
    KalmanFilter();
    void predict(const SensorData &data, double dt);
    void correct(const Eigen::VectorXd &z); 

    const Eigen::VectorXd& getMu() const;
    const Eigen::MatrixXd& getSigma() const;

private:
    Eigen::VectorXd mu_;     // Zustand: [x, y, theta, v, omega]
    Eigen::MatrixXd Sigma_;  // Kovarianzmatrix
    Eigen::MatrixXd R_;      // Prozessrauschen
};

#endif // KALMAN_FILTER_H