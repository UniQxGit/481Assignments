#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;

const double PI = 3.14159265359;
bool isMoving = false;

//Functions
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);

int main(int argc, char ** argv)
{
	//Basic Ros necessities
	ros::init(argc, argv, "shape2");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	//Topics to listen/publish to
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	double radius = 10.0;
	double rotationSpeed = 50; //How to make rotation speed relative to radius?

	/***********************************
	ROTATION TEST
	************************************/
	rotate(degrees2radians(rotationSpeed), degrees2radians(150) , true);
	rotate(degrees2radians(rotationSpeed*4.0), degrees2radians(180) , true);
	rotate(degrees2radians(rotationSpeed*2.0), degrees2radians(300) , false);
	rotate(degrees2radians(rotationSpeed*4.0), degrees2radians(180) , true);
	rotate(degrees2radians(rotationSpeed), degrees2radians(150) , true);

	//extra necessary stuff
	loop_rate.sleep();
	ros::spin();
	return 0;
}


double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x = 1;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}