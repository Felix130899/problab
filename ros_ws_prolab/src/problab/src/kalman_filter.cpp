// kalman_filter.cpp
#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    //Initialize the previous state, cavariance and process noise
    mu_ = Eigen::VectorXd::Zero(6);
    Sigma_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    R_ = Eigen::MatrixXd::Identity(6, 6) * 0.001;
}

void KalmanFilter::predict(const SensorData &data, double dt)
{
    //State matrix A with x,y,theta,v_x,v_y,w
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
    A(0, 3) = dt; // x * dt
    A(1, 4) = dt; // y * dt
    A(2, 5) = dt; // theta * dt

    //Controll transformation matrix
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 2);
    B(3, 0) = 1.0; // v_new = v_old + v_cmd
    B(4, 1) = 1.0; // omega_new = omega_old + omega_cmd

    //Calculating the next step mu_ and the covarianze
    mu_ = A * mu_ + B * data.control;
    Sigma_ = A * Sigma_ * A.transpose() + R_; //  R_ process noise
}


void KalmanFilter::correct(const SensorData &data)
{
    //Sensor input z, from odom sensor v and w
    Eigen::VectorXd z(2);
    z(0) = data.odom_linear_velocity;
    z(1) = data.odom_angular_velocity_z;  
    
    //Measurement matrix C
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2, 6);
    C(0, 3) = 1.0; //odom_v
    C(1, 4) = 1.0; //odom_w

    //Measurement noise Q, calibrated by the user (not dynamic)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2, 2);
    Q(0,0) = 0.005;  // imu_w verrauschter
    Q(1,1) = 0.01;   // odom_w → glatter

    //Kalman-gain
    Eigen::MatrixXd S = C * Sigma_ * C.transpose() + Q;
    Eigen::MatrixXd K = Sigma_ * C.transpose() * S.inverse();

    //Preview calculation 
    Eigen::VectorXd y = z - C * mu_;

    //Correction calculation
    mu_ = mu_ + K * y;
    Sigma_ = (Eigen::MatrixXd::Identity(6, 6) - K * C) * Sigma_;

    // Normalizing theta[-pi, pi]
    mu_(2) = std::fmod(mu_(2) + M_PI, 2 * M_PI);
    if (mu_(2) < 0) mu_(2) += 2 * M_PI;
    mu_(2) -= M_PI;

    //For debugging purposes
    // std::cout << std::fixed << std::setprecision(4);
    // std::cout << "[KalmanFilter] Aktueller Zustand (mu):\n";
    // std::cout << "  x      = " << mu_(0) << "\n";
    // std::cout << "  y      = " << mu_(1) << "\n";
    // std::cout << "  theta  = " << mu_(2) << " rad\n";
    // std::cout << "  v_x    = " << mu_(3) << " m/s\n";
    // std::cout << "  v_y    = " << mu_(4) << " m/s\n";
    // std::cout << "  omega  = " << mu_(5) << " rad/s\n";

}

const Eigen::VectorXd& KalmanFilter::getMu() const { return mu_; }
const Eigen::MatrixXd& KalmanFilter::getSigma() const { return Sigma_; }