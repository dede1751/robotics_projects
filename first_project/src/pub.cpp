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

		/// Reset node state to starting parameters set in launchfile
		void reset_state() {
			n.getParam("odom_node/starting_x", x);
			n.getParam("odom_node/starting_y", y);
			n.getParam("odom_node/starting_th", th);

			velocity = 0.0;
			angle = 0.0;
			last_time = ros::Time::now();

			ROS_INFO("\nRESET TO STARTING PARAMETERS: x=%f, y=%f, th=%f", x, y, th);
		}

		/// Compute the odometry based on previous position and input values
		/// The vehicle is approximated to a bicycle model with a 2.8m wheelbase
		/// The formula used is that of exact odometry, with the assumption of a steering angle
		/// smaller than 90 degrees.
		void compute_odometry()
		{
			ros::Time current_time = ros::Time::now();
			float time_delta = (current_time - last_time).toSec();
			float tan_angle = tan(angle);

			if (tan_angle == 0) {
				//current theta is the same as previous theta
				//there is no osculation circle
				x += velocity * time_delta * cos(th);
				y += velocity * time_delta * sin(th);
			} else {
				float r = WHEEL_BASE / tan_angle;
				float new_th = th + velocity * time_delta / r;

				x += r * (sin(new_th) - sin(th));
				y -= r * (cos(new_th) - cos(th));
				th = new_th;
			}
		}

		/// Publish the odometry in the standard Odometry message
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

		/// Publish the odometry in a custom message
		void publish_custom_odometry()
		{
			first_project::Odom odom;
			odom.x = x;
			odom.y = y;
			odom.th = th;
			odom.timestamp = std::to_string(ros::Time::now().toSec());

			custom_odom_pub.publish(odom);
		}

		/// Publish the odometry in the tf tree
		void publish_tf() {
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(x, y, 0) );
			transform.setRotation(tf::createQuaternionFromYaw(th));

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
		}

	public:
		/// Initialize node by setting up publishers, subscribers and services
		pub()
		{
			reset_state();
			odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
			custom_odom_pub = n.advertise<first_project::Odom>("custom_odometry", 1000);
			srv = n.advertiseService("reset_odom", &pub::reset_callback, this);
			sub = n.subscribe("/speed_steer", 1, &pub::odometry_callback, this);
		}

		/// Callback to be run every time input values are provided on the /speed_steer topic
		void odometry_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
		{
			compute_odometry();

			publish_odometry();
			publish_custom_odometry();
			publish_tf();

			velocity = msg->x;
			angle = msg->y;
			last_time = ros::Time::now();

			ROS_INFO("\nCALLBACK TRIGGERED:\n - ODOM : x=%f y=%f th=%f\n - INPUT: v=%f a=%f", x, y, th, velocity, angle);
		}

		/// Callback to be used by the reset service
		bool reset_callback(
			first_project::ResetOdom::Request  &req,
			first_project::ResetOdom::Response &res
		){
			reset_state();
			res.resetted = true;
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
