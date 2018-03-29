#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "boost/bind.hpp"

#include <sstream>
#include <vector>

using namespace std;
//A lot of this stuff is based from 'wiki.ros.org/turtlesim' youtube video tutorials

//Global Variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
turtlesim::Pose turtlesim_pose;    	//Current position of the turtle
bool poseHasUpdated = false;		//Used to wait for initial pose data before moving
const double PI = 3.14159265359;

//hw3 stuff
struct Turtle {
	void print();
	double h();
	double g();
	double f();
	Turtle()
	{
		parent = NULL;
		name = "";
		type = "";
		hValue = 0;
		gValue = 0;
		fValue = 0;
		pose.x = -1;
		pose.y = -1;
	}

	Turtle(string n)
	{
		parent = NULL;
		name = n;
		type = "";
		hValue = 0;
		gValue = 0;
		fValue = 0;
		pose.x = -1;
		pose.y = -1;
	}

	string name;
	string type;
	double hValue;
	double gValue;
	double fValue;
	ros::Subscriber sub;
	turtlesim::Pose pose;

	Turtle *parent;
	vector<Turtle *> children;
};

class Tree {
public:
	Tree()
	{
		root = Turtle();
		root.name = "ROOT";
		count = 0;
	}

	void add(Turtle *parent,Turtle *node);
	void printat(Turtle t);
	//Turtle[] getPath(Turtle t); //Replace with search function
	int getCount();
	Turtle root;

private:
	int count;
};

bool T_turtlesHasUpdated = false;
int num_turtles_updated = 0;
int num_turtles = 3;
// ros::Subscriber T_subber[num_T_turts];
// turtlesim::Pose T_turtles[num_T_turts] = {};
vector<Turtle> xTurtles;

//Functions
double degrees2radians(double angle_in_degrees);
void rotate (double angular_speed, double relative_angle, double linearSpeed, bool clockwise);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);
void move_goal (turtlesim::Pose goal_pose);

void Turtle::print()
{
	std::cout << "Name: " << name << "\nPARENT: " << (parent!=NULL?parent->name:"NONE") << "\nType: " << type << "\nLocation: (" << pose.x << "," << pose.y << ")" << "\nF(n): " << f() << std::endl;
	std::cout << "Children:" << std::endl;
	for (int i = 0; i < children.size(); i++)
	{
		//children[i].print();
		std::cout << "\tName: " << children[i]->name << "\n\tType: " << children[i]->type << "\n\tLocation: (" << children[i]->pose.x << "," << children[i]->pose.y << ")" << "\n\tF(n): " << f() << "\n" << std::endl;
	}
	if(children.size() == 0)
	{
		std::cout << "\t" << name << " has no children \n" << std::endl;
	}
}

//Replace with heuristic code
double Turtle::h()
{
	hValue = 0;
	return hValue;
}

//Replace with cost code
double Turtle::g()
{
	gValue = 0;
	return gValue;
}

double Turtle::f()
{
	return hValue + gValue;
}

void Tree::add(Turtle *parent, Turtle *node)
{
	node->parent = new Turtle("EMPTY");
	if(parent == NULL)
	{
		node->parent = &root;
		root.children.push_back(node);
	}else{
		parent->children.push_back(node);
		node->parent = parent;
	}
	// std::cout << "\nAdding ";
	// node->print();
}

int Tree::getCount()
{
	return count;
}

void Tree::printat(Turtle t)
{
	t.print();
	
	if(t.children.size() <= 0 )
	{
		return;
	}

	for(int i = 0; i < t.children.size(); i++)
	{
		printat(*t.children[i]);
	}
}


// void show_all_turts() 
// {
// 	for (int i = 0; i < num_T_turts; i < i++) 
// 	{
// 		std::cout << "T_turtle " << i + 1 << ": (" << T_turtles[i].x << ", " << T_turtles[i].y << ")" << std::endl;
// 	}
// }

void GetTurtles()
{

}


void get_all_turts(const turtlesim::Pose::ConstPtr & pose_message, Tree *tree, Turtle *t)
{
	// after the initial update we don't need to worry anymore
	if (!T_turtlesHasUpdated && t->pose.x == -1 && t->pose.y == -1) 
	{

		//std::cout << "Updating " << t->type << " Turtle " << std::endl;
		turtlesim::Pose temp;

		temp.x = pose_message->x;
		temp.y = pose_message->y;
		temp.theta = pose_message->theta;


		t->pose = temp;
		
		if(t->type == "T")
		{
			tree->add(NULL,t);

			//Testing adding to tree
			tree->add(t,new Turtle("TestChild1"));
			tree->add(t,new Turtle("TestChild2"));
		}
		else if(t->type == "X")
			xTurtles.push_back(*t);
		//T_turtles[topic_num - 1] = temp;
	}


	if (num_turtles_updated >= num_turtles || T_turtlesHasUpdated) 
	{
		T_turtlesHasUpdated = true;
	}
	num_turtles_updated++;
}

int main(int argc, char ** argv)
{
	std::cout << "beginning hw3" << std::endl;
	//Basic Ros necessities
	ros::init(argc, argv, "hw3");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	//Topics to listen/publish to
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//subscribe to T's
	ros::master::V_TopicInfo alltopics;
	ros::master::getTopics(alltopics);

	num_turtles = alltopics.size()-1;

	Tree tree;
	Turtle *t;//, *tx;
	bool foundT = false, foundX = false;

	for (int i = 1; i <= alltopics.size(); i++) 
	{

		stringstream topic_stream;
		topic_stream << i;
		string topic_num = topic_stream.str(); 

		stringstream name_stream_T;
		name_stream_T << "T" << i;

		stringstream topic_stream2;
		topic_stream2 << "/T" << i << "/pose";
		string topic_string_T = topic_stream2.str();


		stringstream topic_stream3;
		stringstream name_stream_X;
		name_stream_X << "X" << i;
		topic_stream3 << "/X" << i << "/pose";
		string topic_string_X = topic_stream3.str();

		foundT = false;
		foundX = false;
		
		for(int j = 1; j <= alltopics.size(); j++)
		{
			//std::cout << "Checking : " << topic_string_X << " against " << alltopics[i].name << std::endl;
			if (alltopics[j].name.compare(topic_string_T) == 0 && !foundT)
			{
				foundT = true;
				// std::cout << "Found: " << name_stream_T.str() << std::endl;
				t = new Turtle();
				t->name = name_stream_T.str();
				t->type = "T";
				t->sub = n.subscribe<turtlesim::Pose>(topic_string_T, 10, boost::bind(get_all_turts, _1, &tree, t));
				//T_subber[i] = n.subscribe<turtlesim::Pose>(topic_string, 10, boost::bind(get_all_turts, _1, &t, topic_num));
			}



			if (alltopics[j].name.compare(topic_string_X) == 0 && !foundX)
			{
				foundX = true;
				// std::cout << "Found: " << name_stream_X.str() << std::endl;
				t = new Turtle();
				t->name = name_stream_X.str();
				t->type = "X";
				t->sub = n.subscribe<turtlesim::Pose>(topic_string_X, 10, boost::bind(get_all_turts, _1, &tree, t));
				//T_subber[i] = n.subscribe<turtlesim::Pose>(topic_string, 10, boost::bind(get_all_turts, _1, &t, topic_num));
			}	
		}
		
	}



	//wait for initial 'turtle position' data to come in (absolutely necessary!)
	while(!poseHasUpdated || !T_turtlesHasUpdated)
	{
		ros::spinOnce();
	}

	//hw3 stuff
	//show_all_turts();

	std::cout << "\nPRINTING X TURTLES: " << std::endl;
	for (int i = 0; i < xTurtles.size(); i++)
	{
		//xTurtles[i].print();
		std::cout << "\nName: " << xTurtles[i].name << "\nType: " << xTurtles[i].type << "\nLocation: (" << xTurtles[i].pose.x << "," << xTurtles[i].pose.y << ")" << std::endl;
	}

	std::cout << "\nPRINTING TREE: " << std::endl;
	tree.printat(tree.root);

	//-----Drawing the crown-----
	double height = 3.0;
	double width = 3.0;
	turtlesim::Pose start_pose = turtlesim_pose;
	turtlesim::Pose goal_pose;

	//move up
	//goal_pose.x = start_pose.x;
	//goal_pose.y = start_pose.y + height;
	//move_goal(goal_pose);

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