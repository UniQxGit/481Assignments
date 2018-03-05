#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <sstream>

using namespace std;
//A lot of this stuff is based from 'wiki.ros.org/turtlesim' youtube video tutorials

//Global Variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
turtlesim::Pose turtlesim_pose;    	//Current position of the turtle
bool poseHasUpdated = false;		//Used to wait for initial pose data before moving
const double PI = 3.14159265359;

//Functions
double degrees2radians(double angle_in_degrees);
void rotate (double angular_speed, double relative_angle, double linearSpeed, bool clockwise);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);
void move_goal (turtlesim::Pose goal_pose);

int main(int argc, char ** argv)
{
	//Basic Ros necessities
	ros::init(argc, argv, "shape1");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	//Topics to listen/publish to
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//wait for initial 'turtle position' data to come in (absolutely necessary!)
	while(!poseHasUpdated)
	{
		ros::spinOnce();
	}

	//-----Drawing the crown-----
	double height = 3.0;
	double width = 3.0;
	turtlesim::Pose start_pose = turtlesim_pose;
	turtlesim::Pose goal_pose;

	std::cout << "starting crown" << std::endl;
	//move up
	goal_pose.x = start_pose.x;
	goal_pose.y = start_pose.y + height;
	move_goal(goal_pose);

	//move down+right
	goal_pose.x = start_pose.x + (width / 2.0);
	goal_pose.y = start_pose.y + (height / 2.0);
	move_goal(goal_pose);

	//move up+right
	goal_pose.x = start_pose.x + width;
	goal_pose.y = start_pose.y + height;
	move_goal(goal_pose);

	//move down
	goal_pose.x = start_pose.x + width;
	goal_pose.y = start_pose.y;
	move_goal(goal_pose);

	//go back to spawn point
	move_goal(start_pose);
	std::cout << "finished crown" << std::endl;

	//extra necessary stuff
	loop_rate.sleep();
	ros::spin();
	return 0;
}

//set a pose for the turtle to move to
void move_goal (turtlesim::Pose goal_pose)
{
	geometry_msgs::Twist vel_msg; //The 'Twist' type is a message that the turtle understands
	ros::Rate loop_rate(10);

	//rotate the turtle first
	double current_angle = 0.0;
	double starting_angle = turtlesim_pose.theta;
	double t0 = ros::Time::now().toSec();
	float targetRotation = atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta;
	double angular_speed = degrees2radians(50);

	bool clockwise = (targetRotation<0);
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z = -abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);

	do{
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		std::cout << "Target Rotation: " << (targetRotation*180/PI) 
		<< " Speed: " << (angular_speed*180/PI) 
		<< " CAngle: " << (current_angle*180/PI) 
		<< " DTime:"  << (t1-t0)
		<< std::endl;


		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<(abs(targetRotation))-degrees2radians(6));

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

	//make the turtle walk
	do 
	{
		//Once we reach the desired 'goal_pose', linear velocity goes to 0
		vel_msg.linear.x = 1.0 * getDistance (turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

		//chosen tolerance for linear velocity is 0.05
	} while (getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > .16);
	
	//make the turtle stop
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x = 0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);

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
	vel_msg.linear.x = 0;
	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}



//------callback functions------

//This is how we track the turtle's current position/direction
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	poseHasUpdated = true;
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}

//-------helper functions-------
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2), 2) + pow ((y1-y2),2));
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}