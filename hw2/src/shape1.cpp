#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <sstream>

//A lot of this stuff is based from 'wiki.ros.org/turtlesim' youtube video tutorials

//Global Variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
turtlesim::Pose turtlesim_pose;    	//Current position of the turtle

//Functions
void move_forward (double speed, double distance);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);
void move_goal (turtlesim::Pose goal_pose, double distance_tolerance);

int main(int argc, char ** argv)
{
	//Basic Ros necessities
	ros::init(argc, argv, "shape1");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	//Topics to listen/publish to
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//moving the turtle
	turtlesim::Pose goal_pose;
	goal_pose.x = 1;
	goal_pose.y = 1;
	goal_pose.theta = 0;
	move_goal(goal_pose, 0.01);

	//extra necessary stuff
	loop_rate.sleep();
	ros::spin();
	return 0;
}

//set a pose for the turtle to move to
//	-TODO: make the turtle turn before moving
void move_goal (turtlesim::Pose goal_pose, double distance_tolerance)
{
	geometry_msgs::Twist vel_msg; //The 'Twist' type is a message that the turtle understands
	ros::Rate loop_rate(10);

	do 
	{
		//Once we reach the desired 'goal_pose', linear velocity goes to 0
		vel_msg.linear.x = 1.5 * getDistance (turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		//Once we're pointed in the direction of the 'goal_pose', angular velocity goes to 0
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4 * (atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	} while (getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

	//make the turtle stop
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

//------callback functions------

//This is how we track the turtle's current position/direction
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}

//-------helper functions-------
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2), 2) + pow ((y1-y2),2));
}






//IMPORTANT: just used this for testing/learning, delete this before submitting
//method to move turtle straight
void move_forward (double speed, double distance)
{
	//publish a 'twist' method
	geometry_msgs::Twist vel_msg;

	//linear velocity (this is NOT distance, its all speed)
	vel_msg.linear.x = abs(speed); //positive is forwards, negative is backwards
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//angular velocity
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	//distance = speed * time
	double t0 = ros::Time::now().toSec(); //initial time
	double current_distance = 0;
	ros::Rate loop_rate(10); //10 messages per second

	do 
	{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);


		ros::spinOnce(); //so that publish command runs, rather than just be stored in the buffer
		loop_rate.sleep();

	} while (current_distance < distance); //keep going until we reached our wanted 'distance'

	//force the robot to stop immediately, it will just keep going otherwise
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}
