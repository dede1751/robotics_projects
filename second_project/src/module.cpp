#include "ros/ros.h"
#include "std_msgs/String.h"
#include "first_project/Odom.h"
#include <nav_msgs/Odometry.h>
#include "first_project/ResetOdom.h"
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <sstream>

#define D 2.8
#define PI 3.14

class OdomNode {
private:
    ros::NodeHandle n;
    ros::Subscriber odometry_sub;
    ros::Subscriber laser_scan_sub;
    ros::Subscriber lidar_scan_sub;
    tf::TransformBroadcaster br;
    ros::Timer timer;

public:

    OdomNode() {
        odometry_sub = n.subscribe("t265/odom", 1, &OdomNode::odom_callback, this);
        laser_scan_sub = n.subscribe("/scan", 1, &OdomNode::laser_callback, this);
        lidar_scan_sub = n.subscribe("/converted_scan", 1, &OdomNode::lidar_callback, this);
        stamp = ros::Time::now();
        ROS_INFO("CREATED NODE");
        ROS_INFO("x: %f, y: %f", this->x, this->y);
    }

    void odom_callback(const geometry_msgs::Quaternion::ConstPtr &quat) {
        quaternion = *quat;
        publishTf();
    }

    void laser_callback(const velodyne_points)

    void publishTf() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(this->x, this->y, 0));
        tf::Quaternion q;
        transform.setRotation(tf::createQuaternionFromYaw(this->th));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }


    void laser_callback(const turtlesim::PoseConstPtr &msg) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        transform.setRotation(q);
        this.br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "second_project");

    OdomNode node;

    ros::spin();
    return 0;
}