#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

bool loadWaypoints(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped>& goals)
{
  XmlRpc::XmlRpcValue wp_list;
  if (!nh.getParam("waypoints", wp_list)) {
    ROS_ERROR("Failed to get param 'waypoints'");
    return false;
  }

  if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("'waypoints' param is not an array");
    return false;
  }

  for (int i = 0; i < wp_list.size(); ++i) {
    if (wp_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || wp_list[i].size() != 3) {
      ROS_ERROR("Waypoint %d is not a [x, y, yaw] array", i);
      return false;
    }

    double x = static_cast<double>(wp_list[i][0]);
    double y = static_cast<double>(wp_list[i][1]);
    double yaw = static_cast<double>(wp_list[i][2]);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    goals.push_back(goal);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager");
  ros::NodeHandle nh("~");

  std::vector<geometry_msgs::PoseStamped> waypoints;
  if (!loadWaypoints(nh, waypoints)) {
    ROS_FATAL("Could not load waypoints. Exiting.");
    return 1;
  }

  MoveBaseClient ac("/move_base", true);
  ROS_INFO("Waiting for move_base action server...");
  ac.waitForServer();
  ROS_INFO("Connected to move_base");

  for (size_t i = 0; i < waypoints.size(); ++i) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = waypoints[i];
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal %lu: [%.2f, %.2f, %.2f]", i,
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y,
             tf::getYaw(goal.target_pose.pose.orientation));

    ac.sendGoal(goal);
    bool finished = ac.waitForResult(ros::Duration(60.0));

    if (!finished) {
      ROS_WARN("Goal %lu timed out. Skipping.", i);
      continue;
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal %lu reached successfully.", i);
    } else {
      ROS_WARN("Goal %lu failed: %s", i, ac.getState().toString().c_str());
    }
  }

  ROS_INFO("All waypoints processed.");
  return 0;
}
