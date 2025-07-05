// partical_filter.cpp
#include "partical_filter.h"
#include <random>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



ParticalFilter::ParticalFilter()
    : gen_(std::random_device()()), // Initialize random generator in constructor
      motion_noise_trans_(0.05),    // Initialize member variables
      motion_noise_rot_(0.02),
      sensor_noise_(0.5),
      noise_trans_(0.0, motion_noise_trans_), // Initialize distributions
      noise_rot_(0.0, motion_noise_rot_)
{
    // All initializations done in the initializer list
}

ParticalFilter::ParticalFilter(ros::NodeHandle& nh)
    : gen_(std::random_device()()), // Initialize random generator in constructor
      motion_noise_trans_(0.5),    // Example values, consider loading from ROS parameters
      motion_noise_rot_(0.2),
      sensor_noise_(0.1),
      noise_trans_(0.0, motion_noise_trans_), // Initialize distributions
      noise_rot_(0.0, motion_noise_rot_)
{
    //particles_pub_= nh.advertise<sensor_msgs::PointCloud2>("particles_topic", 1);
    particles_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("particles_marker_topic", 1);
    ROS_INFO("Particle Marker publisher created on topic: %s", particles_marker_pub_.getTopic().c_str());

    // All initializations done in the initializer list
}


void ParticalFilter::init()
{

    //Initalisiert Zufallsgenerator
    std::random_device rd;
    std::mt19937 gen(rd());

    //Standartabweichung für die Gaußche Verteilung
    double sigma_x_intit = 0.1;
    double sigma_y_intit = 0.1;
    double sigma_theta_intit = 0.05;

    //Normalverteilung um 0,0,0
    std::normal_distribution<> d_x_init(0.0, sigma_x_intit);
    std::normal_distribution<> d_y_init(0.0, sigma_y_intit);
    std::normal_distribution<> d_theta_init(0.0, sigma_theta_intit);

    //Löscht noch vorhandene Pratikel und 'particales' ist jetzt eine Membervariable der Klasse ParticalFilter
    particales_.clear();
    particales_.reserve(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle p;
        p.x = d_x_init(gen);
        p.y = d_y_init(gen);
        p.theta = d_theta_init(gen);
        p.weight = 1.0 / NUM_PARTICLES; //Anfangsgewicht gleichmäßig verteilen
        particales_.push_back(p);
    }
}


void ParticalFilter::motionUpdate(double v, double omega, double dt)
{
    for (auto& p : particales_) {
        float v_noisy = v + noise_trans_(gen_);
        float omega_noisy = omega + noise_rot_(gen_);

        float theta = p.theta;
        float delta_theta = omega_noisy * dt;

        float dx, dy;

        if (std::abs(omega_noisy) > 1e-5) {
            float R = v_noisy / omega_noisy;
            dx = R * (std::sin(theta + delta_theta) - std::sin(theta));
            dy = -R * (std::cos(theta + delta_theta) - std::cos(theta));
        } else {
            // Geradeausbewegung
            dx = v_noisy * dt * std::cos(theta);
            dy = v_noisy * dt * std::sin(theta);
        }

        p.x += dx;
        p.y += dy;
        p.theta += delta_theta;
    }

}

void ParticalFilter::updateWeights(const sensor_msgs::LaserScan& scan)
{
    // Bestimme, wie viele Strahlen du nutzen möchtest
    int num_beams_to_use = 10; // Beispiel: Nutze 10 Strahlen

    // Berechne den Schritt, um die Strahlen gleichmäßig zu verteilen
    // Wenn num_beams_to_use = 1, wird nur der mittlere Strahl verwendet.
    // Wenn num_beams_to_use > scan.ranges.size(), verwenden wir alle Strahlen.
    int step_size = 1;
    if (num_beams_to_use > 0 && scan.ranges.size() > num_beams_to_use) 
    {
        step_size = scan.ranges.size() / num_beams_to_use;
    }
    if (step_size == 0) step_size = 1; // Sicherstellen, dass step_size mindestens 1 ist

    //#####################################################################
    for (int i = 0; i < scan.ranges.size(); i += step_size) 
    {
        if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) 
        {
            continue;
        }

        float actual_distance = scan.ranges[i];
        float angle = scan.angle_min + i * scan.angle_increment;

        // ROS_INFO_STREAM("LaserScan: Beam " << i 
        //                 << " | angle = " << angle 
        //                 << " rad | distance = " << actual_distance << " m");
            // Sicherstellen, dass der Scan gültige Daten enthält
        if (scan.ranges.empty()) 
        {
            //ROS_WARN("Received empty laser scan, skipping weight update.");
            return;
        }
    }
    //#########################################################################

    //ROS_INFO_STREAM("ParticalFilter::updateWeights(): Processing scan with " << scan.ranges.size() << " total ranges.");
    //ROS_INFO_STREAM("  Using approximately " << (scan.ranges.size() / step_size) << " beams with step_size " << step_size << ".");


    for (auto& p : particales_) 
    {
        float log_weight = 0.0;


        // Iteriere über eine Auswahl von Strahlen aus dem tatsächlichen LaserScan
        for (int i = 0; i < scan.ranges.size(); i += step_size) 
        {
            // Überspringe ungültige Messungen (NaN oder Inf)
            if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) 
            {
                continue;
            }

            float actual_distance = scan.ranges[i];

            // Berechne den relativen Winkel für diesen spezifischen Laserstrahl
            // scan.angle_min ist der Winkel des ersten Strahls
            // scan.angle_increment ist der Winkelabstand zwischen den Strahlen
            float angle_offset_for_this_beam = scan.angle_min + static_cast<float>(i) * scan.angle_increment;

            // Berechne den absoluten Winkel des Strahls relativ zur Karte
            float beam_theta = p.theta + angle_offset_for_this_beam;

            // Hole die erwartete Distanz von der Karte für diesen Strahl und dieses Partikel
            float expected_distance = getExpectedDistanceFromMap(p.x, p.y, beam_theta);

            if (!std::isfinite(expected_distance) || !std::isfinite(actual_distance)) 
            {
                continue; // Strahl überspringen
            }

            // Vergleiche erwartete und tatsächliche Distanz und berechne die Wahrscheinlichkeit
            float error = expected_distance - actual_distance;
            // Gaußsche Verteilung zur Berechnung der Wahrscheinlichkeit
            // sensor_noise_ steuert die Breite der Verteilung (Toleranz gegenüber Fehlern)
            float prob = std::exp(- (error * error) / (2 * sensor_noise_ * sensor_noise_));

            // Multipliziere das Partikelgewicht mit der Wahrscheinlichkeit dieses Strahls
            log_weight += - (error * error) / (2 * sensor_noise_ * sensor_noise_);


            // Optional: Wenn ein Strahl eine sehr geringe Wahrscheinlichkeit liefert,
            // kann man die Schleife frühzeitig abbrechen, um Rechenzeit zu sparen.
            // if (weight < some_threshold) break;
            // ROS_INFO_STREAM("Beam " << i 
            //     << " | Measured: " << actual_distance 
            //     << " m | Expected: " << expected_distance << " m");
        }
        log_weight = std::max(log_weight, -50.0f);  // clamp to avoid exp(-1000)
        p.weight = std::exp(log_weight);

    }

    // Normalisiere die Gewichte aller Partikel
    double sum_of_weights = 0.0;
    for (const auto& p : particales_) 
    {
        sum_of_weights += p.weight;
        ROS_INFO_STREAM("Sum_of_weights:" << sum_of_weights);
    }

    if (sum_of_weights > 0) {
        for (auto& p : particales_) {
            p.weight /= sum_of_weights;
        }
    } else {
        // Fallback: Wenn alle Gewichte 0 sind (z.B. bei initialer Fehlkonfiguration oder schlechtem Match),
        // verteile die Gewichte gleichmäßig neu, um zu vermeiden, dass der Filter stirbt.
        ROS_WARN("Sum of particle weights is zero. Resetting weights to uniform distribution.");
        for (auto& p : particales_) {
            p.weight = 1.0 / NUM_PARTICLES;
        }
        if (!particales_.empty()) 
        {
            ROS_INFO_STREAM("ParticalFilter::updateWeights(): First particle weight AFTER reset to uniform: " << particales_[0].weight);
        }
    }
}



// void ParticalFilter::resample()
// {

// }

// void ParticalFilter::estimatePose()
// {

// }

void ParticalFilter::publishParticles()
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < particales_.size(); ++i)
    {
        const Particle& p = particales_[i]; // Achten Sie auf die korrekte Schreibweise: Particale

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW; // Bleibt bei ARROW
        marker.action = visualization_msgs::Marker::ADD;

        // Position
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = 0.0; // Wichtig: Z auf 0, um sie auf der Ebene zu halten

        // Orientierung aus theta (Rotation nur um die Z-Achse)
        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta); // Roll=0, Pitch=0, Yaw=p.theta
        marker.pose.orientation = tf2::toMsg(q);

        // Größe: WICHTIGE ANPASSUNG HIER für flachere Pfeile
        marker.scale.x = 0.2;  // Länge des Pfeils
        marker.scale.y = 0.01; // Sehr geringe Breite für "flaches" Aussehen
        marker.scale.z = 0.01; // Sehr geringe Höhe für "flaches" Aussehen

        // Farbe (Grün)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Volle Deckkraft

        marker_array.markers.push_back(marker);
    }

    // Löschen alter Marker: Optional, aber oft nützlich, wenn die Anzahl der Marker variieren könnte
    // marker.action = visualization_msgs::Marker::DELETEALL;
    // marker_array.markers.push_back(marker); // Fügt einen DELETEALL Marker hinzu, der alle alten Marker im Namespace löscht

    particles_marker_pub_.publish(marker_array); // Veröffentlichen Sie das MarkerArray
}

bool ParticalFilter::worldToMap(float x, float y, int& map_x, int& map_y) const 
{
    float origin_x = map_.info.origin.position.x;
    float origin_y = map_.info.origin.position.y;
    float resolution = map_.info.resolution;

    map_x = static_cast<int>((x - origin_x) / resolution);
    map_y = static_cast<int>((y - origin_y) / resolution);

    return (map_x >= 0 && map_x < static_cast<int>(map_.info.width) &&
            map_y >= 0 && map_y < static_cast<int>(map_.info.height));
}

float ParticalFilter::getExpectedDistanceFromMap(float x, float y, float theta) 
{
    float step = 0.05;         // 5 cm Schritte
    float max_range = 5.0;     // maximal 5 m
    float last_valid_r = 0.0;

    for (float r = 0.0; r < max_range; r += step) {
        float scan_x = x + r * std::cos(theta);
        float scan_y = y + r * std::sin(theta);

        int mx, my;
        if (!worldToMap(scan_x, scan_y, mx, my)) {
            //ROS_INFO_STREAM("Ray exited map at r=" << r << " → returning " << last_valid_r);
            return last_valid_r; // außerhalb Karte → vorheriger Wert
        }

        int index = my * map_.info.width + mx;
        if (map_.data[index] > 50) {
            //ROS_INFO_STREAM("Hit obstacle at (" << scan_x << ", " << scan_y << "), r=" << r);
            return r;
        }

        last_valid_r = r;
    }

    //ROS_INFO_STREAM("No obstacle hit, returning max_range=" << max_range);
    return std::numeric_limits<float>::infinity();
    //return max_range;
}

void ParticalFilter::setMap(const nav_msgs::OccupancyGrid& map) 
{
    map_ = map;

    ROS_INFO_STREAM("Map loaded:");
    ROS_INFO_STREAM("  Width: " << map_.info.width);
    ROS_INFO_STREAM("  Height: " << map_.info.height);
    ROS_INFO_STREAM("  Resolution: " << map_.info.resolution);
    ROS_INFO_STREAM("  Origin: (" << map_.info.origin.position.x << ", "
                                  << map_.info.origin.position.y << ")");
}


