#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;
    ROS_INFO("Started Navigation node!");

    std::string file_name;
    nh.getParam("navigation/waypoints", file_name);
    
    ROS_INFO("Opening CSV file: %s", file_name.c_str());
    std::ifstream file(file_name); // use argument for filename since it's not here
    std::string line;
    while (getline(file, line)) {
        ROS_INFO("Processing line in the file: %s", line.c_str());
        std::string value;
        std::vector<double> values;
		std::stringstream str(line);
 
        while (getline(str, value, ','))
            values.push_back(stof(value));

        // setup goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = values[0];
        goal.target_pose.pose.position.y = values[1];
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(values[2]);

        //init a simple action client
        MoveBaseClient ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        ROS_INFO("Sending goal. X: %.3f Y: %.3f YAW: %.3f", values[0], values[1], values[2]);
        ac.sendGoal(goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the waypoint is reached!");
        else
            ROS_INFO("Reaching the waypoint failed! Boo!");

        line.clear();
    }

    return 0;
}
