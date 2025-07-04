// ekf_filter.cpp
#include "ekf_filter.h" // Stelle sicher, dass diese Zeile ganz oben ist

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    mu_ = Eigen::VectorXd::Zero(5);
    Sigma_ = Eigen::MatrixXd::Identity(5, 5) * 0.001;
    R_ = Eigen::MatrixXd::Identity(5, 5) * 0.001; // Initialisiere R
    // Im Konstruktor:
    Q_ = Eigen::MatrixXd::Zero(2, 2);
    Q_(0, 0) = 0.01;  // Winkelgeschwindigkeit (wz) – etwas unsicher
    Q_(1, 1) = 0.1;   // Vorwärtsgeschwindigkeit (v) – typischerweise ungenauer


}

// In Ihrer ExtendedKalmanFilter Klasse
void ExtendedKalmanFilter::predict(const SensorData &data, double dt)
{
   
   // Grenzen definieren (je nach Anwendung anpassen)
    const double v_max = 3.0;      // z. B. 3 m/s
    const double v_min = -3.0;     // Rückwärtsfahrt erlaubt?
    const double omega_max = M_PI; // z. B. 180°/s
    const double omega_min = -M_PI;
    // Extrahieren des aktuellen Zustands für die Lesbarkeit und Jakobische Berechnung
    double x = mu_(0);
    double y = mu_(1);
    double theta = mu_(2);
    double v_current = mu_(3);      // Aktuelle lineare Geschwindigkeit aus dem Zustand
    double omega_current = mu_(4);  // Aktuelle Winkelgeschwindigkeit aus dem dem Zustand

    // Extrahieren der Steuereingaben (angenommen data.control ist [delta_lineare_Geschwindigkeit, delta_Winkel_Geschwindigkeit])
    // Basierend auf Ihrem vorherigen Kommentar: B(3, 0) = 1.0; // v_new = v_old + v_cmd
    double delta_v = data.control(0);
    double delta_omega = data.control(1);




    // Berechne die prognostizierten Geschwindigkeiten basierend auf dem aktuellen Zustand und den Steuereingaben
    // Dies sind die Geschwindigkeiten, die die Bewegung während dieses dt tatsächlich verursachen werden
    double v_predicted = v_current + delta_v;
    double omega_predicted = omega_current + delta_omega;


    v_predicted = std::max(v_min, std::min(v_max, v_predicted));
    omega_predicted = std::max(omega_min, std::min(omega_max, omega_predicted));

    // --- 1. Nicht-lineare Prädiktion des Zustands (mu_ = g(mu_prev, u, dt)) ---
    // Dies ersetzt die lineare 'mu_ = A * mu_ + B * data.control;' Zeile.
    // Die Positionen (x, y) hängen nicht-linear von theta ab.
    mu_(0) = x + v_predicted * cos(theta) * dt;
    mu_(1) = y + v_predicted * sin(theta) * dt;

    // Die Orientierung (theta) aktualisiert sich linear
    mu_(2) = theta + omega_predicted * dt;

    // Die Geschwindigkeiten aktualisieren sich basierend auf den Steuereingaben
    mu_(3) = v_predicted;
    mu_(4) = omega_predicted;

    // Optional: Normalisiere den Orientierungswinkel theta auf [-pi, pi]
    // Dies verhindert, dass der Winkel überläuft und numerische Probleme verursacht.
    mu_(2) = std::fmod(mu_(2) + M_PI, 2 * M_PI);
    if (mu_(2) < 0) mu_(2) += 2 * M_PI;
    mu_(2) -= M_PI;


    // --- 2. Berechnung der Jakobischen Matrix des Zustandsübergangs (G) ---
    // Dies ersetzt die lineare 'Eigen::MatrixXd A = ...' und die Verwendung von A zur Kovarianz-Propagierung.
    // G = dg/d(mu) ausgewertet am aktuellen Zustand (mu_ vor der nicht-linearen Aktualisierung)
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(5, 5); // Initialisiere als Einheitsmatrix

    // Fülle die nicht-identischen Terme (Ableitungen von g_i nach mu_j)
    // Diese Terme kommen von der Differenzierung der nicht-linearen Zustandsprädiktionsgleichungen:
    // x_neu = x + (v_alt + delta_v) * cos(theta_alt) * dt
    // y_neu = y + (v_alt + delta_v) * sin(theta_alt) * dt
    // theta_neu = theta_alt + (omega_alt + delta_omega) * dt
    // v_neu = v_alt + delta_v
    // omega_neu = omega_alt + delta_omega

    // Ableitungen für x_neu (Reihe 0 von G)
    G(0, 2) = -v_predicted * sin(theta) * dt; // d(x_neu)/d(theta_alt)
    G(0, 3) = cos(theta) * dt;                // d(x_neu)/d(v_alt)

    // Ableitungen für y_neu (Reihe 1 von G)
    G(1, 2) = v_predicted * cos(theta) * dt;  // d(y_neu)/d(theta_alt)
    G(1, 3) = sin(theta) * dt;                // d(y_neu)/d(v_alt)

    // Ableitungen für theta_neu (Reihe 2 von G)
    G(2, 4) = dt;                             // d(theta_neu)/d(omega_alt)

    // Für v_neu und omega_neu (Reihen 3 und 4 von G) sind die Ableitungen bzgl.
    // x, y, theta gleich 0, und bzgl. v_alt und omega_alt gleich 1.
    // Diese sind bereits durch die Initialisierung der Einheitsmatrix abgedeckt.


    // --- 3. Aktualisierung der Kovarianzmatrix (Sigma_) ---
    // Sigma_ = G * Sigma_ * G.transpose() + R_
    // Dies ersetzt die Zeile 'Sigma_ = A * Sigma_ * A.transpose() + R_;'
    Sigma_ = G * Sigma_ * G.transpose() + R_; // R_ ist Ihre Prozessrauschkovarianzmatrix
}


// In Ihrer ExtendedKalmanFilter Klasse
void ExtendedKalmanFilter::correct(const SensorData &data) 
{

    // --- 1. Messwerte extrahieren ---
    double measured_wz = data.imu_msg_ptr->angular_velocity.z;
    double measured_v = data.linear_velocity;

    // --- 2. Messvektor z ---
    Eigen::VectorXd z(2);
    z(0) = measured_wz;
    z(1) = measured_v;

    // --- 3. Erwartete Messung h(mu) ---
    Eigen::VectorXd h_predicted(2);
    h_predicted(0) = mu_(4); // erwartetes wz = omega
    h_predicted(1) = mu_(3); // erwartetes v

    // --- 4. Innovationsvektor ---
    Eigen::VectorXd y = z - h_predicted;

    // --- 5. Beobachtungsmatrix H ---
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 4) = 1.0; // dh0/d(omega)
    H(1, 3) = 1.0; // dh1/d(v)

    // --- 6. Innovationskovarianz S ---
    Eigen::MatrixXd S = H * Sigma_ * H.transpose() + Q_; // Q_ muss 2x2 sein

    // --- 7. Kalman-Gain ---
    Eigen::MatrixXd K = Sigma_ * H.transpose() * S.inverse();

    // --- 8. Zustand aktualisieren ---
    mu_ = mu_ + K * y;

    // --- 9. Kovarianz aktualisieren ---
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    Sigma_ = (I - K * H) * Sigma_;

    // --- 10. Winkel normalisieren ---
    mu_(2) = std::fmod(mu_(2) + M_PI, 2 * M_PI);
    if (mu_(2) < 0) mu_(2) += 2 * M_PI;
    mu_(2) -= M_PI;
}


const Eigen::VectorXd& ExtendedKalmanFilter::getMu() const { return mu_; }
const Eigen::MatrixXd& ExtendedKalmanFilter::getSigma() const { return Sigma_; }