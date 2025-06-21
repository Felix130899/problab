#pragma once
#include <Eigen/Dense>
#include "convert_sensor_data.h"

class KalmanFilter {
public:
    KalmanFilter();
    void predict(const SensorData &data, double dt);
    void correct(const Eigen::VectorXd &z);  // Platzhalter für später
    const Eigen::VectorXd& getMu() const;
    const Eigen::MatrixXd& getSigma() const;

private:
    Eigen::VectorXd mu_;      // Zustand: [x, y, theta, v, omega]
    Eigen::MatrixXd Sigma_;   // Kovarianzmatrix
    Eigen::MatrixXd R_;       // Prozessrauschen
};
