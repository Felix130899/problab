// ekf_filter.cpp
#include "ekf_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    //Initiliazing
    //Differences to KF mu_p (just for better understanding), 5x5 not 6x6 (just used 6x6 for KF to see if it makes it better, Spoiler: It didn't) 
    mu_ = Eigen::VectorXd::Zero(5);
    mu_p = Eigen::VectorXd::Zero(5);
    Sigma_ = Eigen::MatrixXd::Identity(5, 5) * 0.001;
    R_ = Eigen::MatrixXd::Identity(5, 5) * 0.001;   
    Q_ = Eigen::MatrixXd::Zero(3, 3);
    Q_(0,0) = 0.005;  //imv_w
    Q_(1,1) = 0.01;   //odom_w
    Q_(2,2) = 0.1;    //odom_v
}

void ExtendedKalmanFilter::predict(const SensorData &data, double dt)
{
   
    //Set the borders for realistic driving (O propably don't need it anymore, because the filter is stable but in the beginning it was usefull)
    const double v_max = 3.0;
    const double v_min = -3.0;
    const double omega_max = M_PI;
    const double omega_min = -M_PI;
    
    //Extracting for better readabillity and Jacobian
    double x = mu_(0);
    double y = mu_(1);
    double theta = mu_(2);
    double v_current = mu_(3);      
    double omega_current = mu_(4); 

    //Extracting controller input via convert function
    double delta_v = data.control(0);
    double delta_omega = data.control(1);

    double v_predicted = delta_v;
    double omega_predicted = delta_omega;

    v_predicted = std::max(v_min, std::min(v_max, v_predicted));
    omega_predicted = std::max(omega_min, std::min(omega_max, omega_predicted));

    //Nonlinear prediction of the state (mu_ = g(mu_prev, u, dt))
    //Instead of linear modle from KF 'mu_ = A * mu_ + B * data.control;'
    mu_p(0) = x + v_predicted * cos(theta) * dt;
    mu_p(1) = y + v_predicted * sin(theta) * dt;
    mu_p(2) = theta + omega_predicted * dt;

    //Speed refresh based on controler input
    mu_p(3) = v_predicted;
    mu_p(4) = omega_predicted;

    //Normalize theta [-pi, pi]
    mu_p(2) = std::fmod(mu_p(2) + M_PI, 2 * M_PI);
    if (mu_p(2) < 0) mu_p(2) += 2 * M_PI;
    mu_p(2) -= M_PI;

    //Calculation of the Jacobian (G)
    //Instead of the state matrix A in the KF 'Eigen::MatrixXd A = ...'
    // G = dg/d(mu) 
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(5, 5);

    //Derivation for x_neu (Row 0 of G)
    G(0, 2) = -v_predicted * sin(theta) * dt; // d(x_neu)/d(theta_alt)
    G(0, 3) = cos(theta) * dt;                // d(x_neu)/d(v_alt)

    //Derivation for y_neu (Row 1 of G)
    G(1, 2) = v_predicted * cos(theta) * dt;  // d(y_neu)/d(theta_alt)
    G(1, 3) = sin(theta) * dt;                // d(y_neu)/d(v_alt)

    //Derivation of theta_neu (Row 2 of G)
    G(2, 4) = dt;                             // d(theta_neu)/d(omega_alt)

    //Refreshing of  the covariance(Sigma_)
    //Sigma_ = G * Sigma_ * G.transpose() + R_
    //Insteas of 'Sigma_ = A * Sigma_ * A.transpose() + R_;' in the KF
    Sigma_ = G * Sigma_ * G.transpose() + R_;
}

void ExtendedKalmanFilter::correct(const SensorData &data) 
{

    //Measurements from convert function
    Eigen::VectorXd z(3);
    z(0) = data.imu_angular_velocity_z;
    z(1) = data.odom_angular_velocity_z;
    z(2) = data.odom_linear_velocity;

    //Predicted measurement h(mu)
    Eigen::VectorXd h_predicted(3);
    h_predicted(0) = mu_p(4); // omega
    h_predicted(1) = mu_p(4); // omega
    h_predicted(2) = mu_p(3); // v

    //Matrix H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 5);
    H(0,4) = 1.0; // imu_wz
    H(1,4) = 1.0; // odom_wz
    H(2,3) = 1.0; // odom_v

    //Kalman-Gain
    Eigen::MatrixXd S = H * Sigma_ * H.transpose() + Q_;
    Eigen::MatrixXd K = Sigma_ * H.transpose() * S.inverse();

    //Refresh state
    Eigen::VectorXd y = z - h_predicted;
    mu_ = mu_p + K * y;

    //Refresh covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    Sigma_ = (I - K * H) * Sigma_;

    // Normaliaze theta
    mu_(2) = std::fmod(mu_(2) + M_PI, 2 * M_PI);
    if (mu_(2) < 0) mu_(2) += 2 * M_PI;
    mu_(2) -= M_PI;


    //For Debugging purposes
    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "---------Kalmangain---------" << std::endl;
    // std::cout << K << std::endl;
    // std::cout << "############################" << std::endl;

    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "---------Z---------" << std::endl;
    // std::cout << z << std::endl;
    // std::cout << "############################" << std::endl;

    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "---------h()---------" << std::endl;
    // std::cout << h_predicted << std::endl;
    // std::cout << "############################" << std::endl;

    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "---------mu_ x---------" << std::endl;
    // std::cout << mu_(0) << std::endl;
    // std::cout << "############################" << std::endl;

    //     std::cout << std::fixed << std::setprecision(6);
    // std::cout << "---------mu_ y---------" << std::endl;
    // std::cout << mu_(1) << std::endl;
    // std::cout << "############################" << std::endl;
    
}


const Eigen::VectorXd& ExtendedKalmanFilter::getMu() const { return mu_; }
const Eigen::MatrixXd& ExtendedKalmanFilter::getSigma() const { return Sigma_; }