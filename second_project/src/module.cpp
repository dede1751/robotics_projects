#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <sstream>

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
        odometry_sub = n.subscribe("t265/odom", 1000, &OdomNode::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        geometry_msgs::TransformStamped position;
        position.child_frame_id = "base_footprint";

        position.header.stamp = odom_msg->header.stamp;
        position.header.frame_id = odom_msg->header.frame_id;


        position.transform.translation.x = odom_msg->pose.pose.position.x;
        position.transform.translation.y = odom_msg->pose.pose.position.y;
        position.transform.translation.z = odom_msg->pose.pose.position.z;

        position.transform.rotation.w = odom_msg->pose.pose.orientation.w;
        position.transform.rotation.x = odom_msg->pose.pose.orientation.x;
        position.transform.rotation.y = odom_msg->pose.pose.orientation.y;
        position.transform.rotation.z = odom_msg->pose.pose.orientation.z;


        br.sendTransform(position);
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_publisher");

    OdomNode node;

    ros::spin();
    return 0;
}