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
        odometry_sub = n.subscribe("t265/odom", 1, &OdomNode::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {

        nav_msgs::Odometry msg = *odom_msg;

        tf::Transform transform;


        transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
        transform.setRotation(tf::createQuaternionFromYaw(msg.pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "t265"));
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_publisher");

    OdomNode node;

    ros::spin();
    return 0;
}