#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    mu_ = Eigen::VectorXd::Zero(5);
    Sigma_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;
    R_ = Eigen::MatrixXd::Identity(5, 5) * 0.01;
}

void KalmanFilter::predict(const SensorData &data, double dt)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(5, 5);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 4) = dt;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(5, 2);
    B(3, 0) = 1.0;
    B(4, 1) = 1.0;

    mu_ = A * mu_ + B * data.control;
    Sigma_ = A * Sigma_ * A.transpose() + R_;
}

void KalmanFilter::correct(const Eigen::VectorXd &z)
{
    // Noch leer – kommt später
}

const Eigen::VectorXd& KalmanFilter::getMu() const { return mu_; }
const Eigen::MatrixXd& KalmanFilter::getSigma() const { return Sigma_; }
