#include <ros/ros.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv) {

    ros::init(argc, argv, "navigation");
    ROS_INFO("Starting node");
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();


    std::ifstream file("/wp1.csv");

    int j = 0;


    while (file) {

        std::string delimiter = ",";

        size_t pos = 0;
        std::string token;
        std::string s;
        file >> s;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            s.erase(0, pos + delimiter.length());

            ROS_INFO("sium");
            switch (j) {
                case 0:
                    goal.target_pose.pose.position.x = stoi(token);
                    j = 1;
                    break;
                case 1:
                    goal.target_pose.pose.orientation.y = stoi(token);
                    j = 2;
                    break;
                case 2:
                    j = 0;
                    goal.target_pose.pose.orientation.w = stoi(token);
                    break;
            }
        }
    }


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    return 0;
}
