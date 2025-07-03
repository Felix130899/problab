// kalman_filter.cpp
#include "kalman_filter.h" // Stelle sicher, dass diese Zeile ganz oben ist

KalmanFilter::KalmanFilter()
{
    mu_ = Eigen::VectorXd::Zero(5);
    Sigma_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;
    R_ = Eigen::MatrixXd::Identity(5, 5) * 0.001; // Initialisiere R
}

void KalmanFilter::predict(const SensorData &data, double dt)
{
    // Deine aktuelle (lineare) Prädiktionslogik für mu_ und Sigma_
    // Diese sollte so bleiben, wie du sie für dein Verständnisprojekt beabsichtigt hast.
    // Wenn du eine A-Matrix hast, die z.B. 
    // A(0, 3) = dt; A(1, 4) = dt; A(2, 4) = dt;
    // beinhaltet, dann bleibt diese bestehen.
    // Bedenke, dass diese lineare Approximation die Form der Kovarianz
    // aufgrund von Rotationen nicht korrekt beeinflusst.

    // Beispiel für die lineare Prädiktion (wie in deinem originalen, auskommentierten Code):
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(5, 5);
    A(0, 3) = dt; // x += v * dt
    A(1, 4) = dt; // y += omega * dt (wenn y von omega abhängt, sonst von v)
    A(2, 4) = dt; // theta += omega * dt

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(5, 2);
    B(3, 0) = 1.0; // v_new = v_old + v_cmd
    B(4, 1) = 1.0; // omega_new = omega_old + omega_cmd

    mu_ = A * mu_ + B * data.control;
    Sigma_ = A * Sigma_ * A.transpose() + R_; // Hier kommt R_ (Prozessrauschen) ins Spiel
}


void KalmanFilter::correct(const Eigen::VectorXd &z)
{
    // Beobachtungsmatrix H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 3) = 1.0; // v (linear velocity)
    H(1, 4) = 1.0; // ω (angular velocity)



    // Hier wird Q als feste Matrix verwendet, nicht dynamisch von der Odometrie
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2) * 0.005; // Beispiel für einen festen Q-Wert

    // Innovation
    Eigen::VectorXd y = z - H * mu_;

    // Kalman-Gain
    Eigen::MatrixXd S = H * Sigma_ * H.transpose() + Q;
    Eigen::MatrixXd K = Sigma_ * H.transpose() * S.inverse();

    // Zustand aktualisieren
    mu_ = mu_ + K * y;
    Sigma_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * Sigma_;

    // Optional: Normiere den Orientierungswinkel theta auf [-pi, pi]
    mu_(2) = std::fmod(mu_(2) + M_PI, 2 * M_PI);
    if (mu_(2) < 0) mu_(2) += 2 * M_PI;
    mu_(2) -= M_PI;
}

const Eigen::VectorXd& KalmanFilter::getMu() const { return mu_; }
const Eigen::MatrixXd& KalmanFilter::getSigma() const { return Sigma_; }