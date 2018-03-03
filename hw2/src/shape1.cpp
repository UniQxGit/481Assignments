#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <sstream>

//A lot of this stuff is based from 'wiki.ros.org/turtlesim' youtube video tutorials

//Global Variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
turtlesim::Pose turtlesim_pose;    	//Current position of the turtle
bool poseHasUpdated = false;		//Used to wait for initial pose data before moving

//Functions
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
	do 
	{
		//The closer we are pointed in the direction of the 'goal_pose', angular velocity goes closer to 0
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 1.0 * (atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

		
		//chosen tolerance for angular velocity is 0.001
	} while (vel_msg.angular.z >= 0.001 || vel_msg.angular.z <= -0.001);

	vel_msg.angular.z = 0;

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
	} while (getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > 0.05);
	
	//make the turtle stop
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
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
