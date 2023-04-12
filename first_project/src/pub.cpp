#include <sstream>
#include <cmath>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#include "first_project/Odom.h"
#include "first_project/ResetOdom.h"

class pub {

	private:
		const float WHEEL_BASE = 2.8;
		float x;
		float y;
		float th;
		float velocity;
		float angle;
		ros::Time last_time;

		ros::NodeHandle n; 
		ros::Publisher odom_pub;
		ros::Publisher custom_odom_pub;
  		tf::TransformBroadcaster br;
		ros::ServiceServer srv;
		ros::Subscriber sub;

		void reset_state() {
			n.getParam("/starting_x", x);
			n.getParam("/starting_y", y);
			n.getParam("/starting_th", th);

			velocity = 0.0;
			angle = 0.0;
			last_time = ros::Time::now();
		}

		void compute_odometry()
		{
			ros::Time current_time = ros::Time::now();
			float time_delta = (current_time - last_time).toSec();
			float tan_angle = tan(angle);

			if (tan_angle == 0) {
				x += velocity * time_delta;
			} else {
				float r = tan_angle / WHEEL_BASE;
				float new_th = th + velocity * r * time_delta;

				x += (sin(new_th) - sin(th)) / r;
				y -= (cos(new_th) - cos(th)) / r;
				th = new_th;
			}
		}

		void publish_odometry()
		{
			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

			odom_pub.publish(odom);
		}

		void publish_custom_odometry()
		{
			first_project::Odom odom;
			odom.x = x;
			odom.y = y;
			odom.th = th;
			odom.timestamp = std::to_string(ros::Time::now().toSec());

			custom_odom_pub.publish(odom);
		}

		void publish_tf() {
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(x, y, 0) );
			transform.setRotation(tf::createQuaternionFromYaw(th));

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
		}

	public:
		pub()
		{
			odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
			custom_odom_pub = n.advertise<first_project::Odom>("custom_odometry", 1000);
			srv = n.advertiseService("reset_odom", &pub::reset, this);
			sub = n.subscribe("/speed_steer", 1, &pub::callback, this);
			reset_state();
		}

		void callback(const geometry_msgs::Quaternion::ConstPtr& msg)
		{
			compute_odometry();

			publish_odometry();
			publish_custom_odometry();
			publish_tf();

			velocity = msg->x;
			angle = msg->y;
			last_time = ros::Time::now();

			ROS_INFO("CALLBACK TRIGGERED: x=%f, y=%f, th=%f, v=%f, a=%f", x, y, th, velocity, angle);
		}

		bool reset(
			first_project::ResetOdom::Request  &req,
			first_project::ResetOdom::Response &res
		){
			reset_state();

			res.resetted = true;
			ROS_INFO("RESET TO START COORDINATES");
			return true;
		}

};

int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "odom_node"); 
	pub my_pub;
	ros::spin();
	return 0;
}
