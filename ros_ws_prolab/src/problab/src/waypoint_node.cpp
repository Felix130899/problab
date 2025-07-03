#include <ros/ros.h>
#include <yaml-cpp/yaml.h> // Für das Laden von YAML-Dateien
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h> // Für Quaternion-Transformationen
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WaypointManager
{
public:
    WaypointManager();
    void sendNextWaypoint(const ros::TimerEvent& event);
    void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void publishWaypointMarkers();

private:
    ros::NodeHandle nh_;
    std::vector<std::vector<double>> waypoints_;
    MoveBaseClient ac_;
    int current_waypoint_index_;
    bool goal_sent_;
    ros::Timer timer_;

    // NEU: Publisher für Waypoint-Visualisierung
    ros::Publisher waypoint_marker_pub_;
};

WaypointManager::WaypointManager() :
    nh_("~"), // NodeHandle mit privatem Namespace für Parameter
    ac_("move_base", true), // True tells the action client that we want to spin a thread by default
    current_waypoint_index_(0),
    goal_sent_(false)
{
    // Warte auf den move_base Action Server
    ROS_INFO("Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("Connected to move_base server.");

    // Waypoints aus dem Parameter-Server laden
    XmlRpc::XmlRpcValue waypoint_list;
    if (nh_.getParam("waypoints", waypoint_list))
    {
        ROS_ASSERT(waypoint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < waypoint_list.size(); ++i)
        {
            ROS_ASSERT(waypoint_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(waypoint_list[i].size() >= 2); // Minimum x, y
            std::vector<double> wp;
            for (int j = 0; j < waypoint_list[i].size(); ++j)
            {
                // XMLRPC values are tricky, ensure correct type casting
                if (waypoint_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    wp.push_back(static_cast<double>(static_cast<int>(waypoint_list[i][j])));
                }
                else if (waypoint_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    wp.push_back(static_cast<double>(waypoint_list[i][j]));
                }
                else
                {
                    ROS_WARN("Waypoint parameter has unexpected type at index [%d][%d]. Skipping.", i, j);
                    wp.push_back(0.0); // Default to 0.0 to avoid crash
                }
            }
            waypoints_.push_back(wp);
        }
    }
    else
    {
        ROS_ERROR("Failed to load waypoints from parameter server!");
        ros::shutdown();
        return;
    }

    if (waypoints_.empty())
    {
        ROS_ERROR("No waypoints loaded. Shutting down.");
        ros::shutdown();
        return;
    }

    // NEU: Initialisierung des Publishers für Waypoint-Marker
    waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoints_markers", 1);
    publishWaypointMarkers(); // Publizieren Sie die Marker sofort

    // Timer, um regelmäßig das nächste Ziel zu senden
    timer_ = nh_.createTimer(ros::Duration(1.0), &WaypointManager::sendNextWaypoint, this);
}

void WaypointManager::publishWaypointMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        const auto& wp = waypoints_[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Wichtig: Frame der Karte
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW; // Oder CUBE, SPHERE, TEXT_VIEW_FACING
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = wp[0];
        marker.pose.position.y = wp[1];
        marker.pose.position.z = 0.1; // Leicht über dem Boden, damit es sichtbar ist

        // Orientierung für Pfeile (optional, nur für ARROW Type relevant)
        if (wp.size() == 3) // Wenn auch eine Gier-Richtung (Yaw) gegeben ist
        {
            tf::Quaternion q;
            q.setRPY(0, 0, wp[2]); // Roll, Pitch, Yaw
            tf::quaternionTFToMsg(q, marker.pose.orientation);
        }
        else // Standard-Orientierung
        {
            marker.pose.orientation.w = 1.0;
        }

        marker.scale.x = 0.5; // Länge des Pfeils/Größe des Markers
        marker.scale.y = 0.1; // Breite des Pfeils
        marker.scale.z = 0.1; // Höhe des Pfeils

        marker.color.a = 1.0; // Alpha (Deckkraft)
        marker.color.r = 0.0;
        marker.color.g = 1.0; // Grün
        marker.color.b = 0.0;

        // Marker für den aktuell angefahrenen Waypoint hervorheben
        if (static_cast<int>(i) == current_waypoint_index_)
        {
            marker.color.r = 1.0; // Rot für aktuellen Waypoint
            marker.color.g = 0.0;
            marker.scale.x = 0.7;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
        }

        marker_array.markers.push_back(marker);
    }
    
    waypoint_marker_pub_.publish(marker_array);
}

void WaypointManager::sendNextWaypoint(const ros::TimerEvent& event)
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        if (!goal_sent_)
        {
            const auto& waypoint = waypoints_[current_waypoint_index_];
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map"; // MUSS "map" sein
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = waypoint[0];
            goal.target_pose.pose.position.y = waypoint[1];
            goal.target_pose.pose.position.z = 0.0;

            // Quaternion für Orientierung (yaw)
            if (waypoint.size() == 3)
            {
                tf::Quaternion q;
                q.setRPY(0, 0, waypoint[2]); // Roll, Pitch, Yaw
                tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);
            }
            else // Standard-Orientierung, wenn kein Yaw gegeben ist
            {
                goal.target_pose.pose.orientation.w = 1.0;
            }

            ROS_INFO("Sending goal to waypoint %d: (%.2f, %.2f)",
                     current_waypoint_index_, waypoint[0], waypoint[1]);
            ac_.sendGoal(goal,
                         boost::bind(&WaypointManager::goalDoneCallback, this, _1, _2));
            goal_sent_ = true;
            publishWaypointMarkers(); // Marker aktualisieren, um aktuellen WP hervorzuheben
        }
    }
    else
    {
        if (!goal_sent_) // Check to avoid redundant log messages
        {
            ROS_INFO("All waypoints reached!");
            // Optional: Roboter stoppen oder in Ruhezustand versetzen
            // ac_.cancelAllGoals();
            // ros::shutdown(); // Kann den Node beenden, wenn alle Waypoints erreicht sind
            goal_sent_ = true; // Markieren, dass alle Ziele gesendet wurden
            current_waypoint_index_ = waypoints_.size(); // Sicherstellen, dass Index außerhalb des Bereichs ist
            publishWaypointMarkers(); // Final update for markers
        }
    }
}

void WaypointManager::goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    goal_sent_ = false;
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Waypoint %d reached!", current_waypoint_index_);
        current_waypoint_index_++;
        // sendNextWaypoint wird durch den Timer aufgerufen
    }
    else
    {
        ROS_WARN("Waypoint %d failed with status: %s", current_waypoint_index_, state.toString().c_str());
        // Hier könnten Sie eine Fehlerbehandlung implementieren, z.B. erneut versuchen oder überspringen
        current_waypoint_index_++; // Trotzdem zum nächsten Waypoint gehen
    }
    publishWaypointMarkers(); // Marker aktualisieren, falls erfolgreich oder fehlgeschlagen
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_manager");
    WaypointManager wm;
    ros::spin();
    return 0;
}