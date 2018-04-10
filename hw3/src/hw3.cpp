#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Kill.h>
#include "boost/bind.hpp"

#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;



class Vector2 {
public:
	Vector2(){}
	Vector2(double x, double y){
		xVal = x;
		yVal = y;
	}
	void set(double x, double y);
	Vector2 normalize();
	double x(){
		return xVal;
	};
	
	double y(){
		return yVal;
	};

	Vector2 operator-(const Vector2 &rhs) const{
		Vector2 result;
		result.set(xVal-rhs.xVal,yVal-rhs.yVal);
		return result;
	}
	double operator*(const Vector2 &rhs) const{
		return xVal*rhs.xVal+yVal*rhs.yVal;
	}
private:
	double xVal;
	double yVal;
};

//hw3 stuff
struct Turtle {
	void print();
	double h();
	double g();
	double f();
	
	Turtle()
	{
		parent = NULL;
		name = "EMPTY";
		type = "NONE";
		hValue = 0;
		gValue = 0;
		fValue = 0;
		pose.x = -1;
		pose.y = -1;
	}

	Turtle(Vector2 pos,Turtle &parent)
	{
		position = pos;
		parent = parent;
		name = "EMPTY";
		type = "NONE";
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
	Vector2 position;


	Turtle *parent;
	vector<Turtle *> children;
	vector<Turtle> xInTheWay;
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
	void setPath();
	vector<Turtle *> getPath();
	void SetAvoidPath();
	int getCount();
	Turtle root;
private:
	int count;
	vector<Turtle *> path;
};

//Global Variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
bool poseHasUpdated = false;		//Used to wait for initial pose data before moving
const double PI = 3.14159265359;
bool T_turtlesHasUpdated = false;
int num_Xturtles_updated = 0;
int num_Tturtles_updated = 0;
int num_Xturtles = 0;
int num_Tturtles = 0;

vector<Turtle> xTurtles;			
Turtle navTurtle; 					//Main turtle to track.
double total_distance = 0;			//total distance traveled
double time_it_took_walking = 0;	//total time it took when turtle was actually walking
ros::ServiceClient kClient;			//service client for killing turtles

//Functions
double degrees2radians(double angle_in_degrees);
void rotate (double angular_speed, double relative_angle, double linearSpeed, bool clockwise);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);
void move_goal (turtlesim::Pose goal_pose);

void Vector2::set(double x, double y)
{
	xVal = x;
	yVal = y;
}

Vector2 Vector2::normalize()
{
	Vector2 normalized;
	double magnitude = sqrt(xVal * xVal + yVal * yVal);
	normalized.set(xVal/magnitude,yVal/magnitude);
	return normalized;
}

void Turtle::print()
{
	std::cout << "Name: " << name << "\nPARENT: " << (parent!=NULL?parent->name:"NONE") << "\nType: " << type << "\nLocation: (" << position.x() << "," << position.y() << ")" << "\nF(n): " << f() << std::endl;
	std::cout << "Children:" << std::endl;
	for (int i = 0; i < xInTheWay.size(); i++)
	{
		std::cout << "Turtles In the way: " << xInTheWay[i].name << "(" << xInTheWay[i].position.x() << "," << xInTheWay[i].position.y() << ")" << endl;
	}

	for (int i = 0; i < children.size(); i++)
	{
		std::cout << "\tName: " << children[i]->name << "\n\tType: " << children[i]->type << "\n\tLocation: (" << children[i]->position.x() << "," << children[i]->position.y() << ")" << "\n\tF(n): " << children[i]->f() << "\n" << std::endl;
	}

	if(children.size() == 0)
	{
		std::cout << "\t" << name << " has no children \n" << std::endl;
	}
}

//Replace with heuristic code
double Turtle::h()
{
	if(parent == NULL)
		return 0;

	double sum = 0.0;
	double dot1 = 0.0, dot2 = 0.0;
	xInTheWay.clear();
	Turtle current = *parent;
	double dist = getDistance(current.position.x(),current.position.y(),position.x(),position.y());
	for (int i = 0; i < xTurtles.size(); i++) {
		//cout << name << ". Checking " << xTurtles[i].name << endl;
		if (((xTurtles[i].position.x() <= current.position.x()+0.5 && xTurtles[i].position.x() >= position.x()-0.5) || (xTurtles[i].position.x() >= current.position.x()-0.5 && xTurtles[i].position.x() <= position.x()+0.5)) &&
			((xTurtles[i].position.y() <= current.position.y()+0.5 && xTurtles[i].position.y() >= position.y()-0.5) || (xTurtles[i].position.y() >= current.position.y()-0.5 && xTurtles[i].position.y() <= position.y()+0.5))) {
			dot1 = (xTurtles[i].position-current.position).normalize() * (position-current.position).normalize();
			dot2 = (xTurtles[i].position-position).normalize() * (current.position-position).normalize();
			
			double angleThreshold = dist/2/(sqrt(0.5*0.5+dist/2*dist/2)); //cos(angle between 2 vectors)=dot product/magnitude of vector1*magnitude of vector2
			if(dot1 > angleThreshold || dot2 > angleThreshold)
			{
				sum += max(dot1, dot2);
				cout << xTurtles[i].name << " in the way: " << xTurtles[i].name <<"(" << xTurtles[i].position.x() << "," << xTurtles[i].position.y() << ") "<< current.name << "(" << current.position.x() << "," << current.position.y() << ") " << name << "(" << position.x() << "," << position.y() << ")" << max(dot1, dot2) << " threshold " << angleThreshold << endl;
				xInTheWay.push_back(xTurtles[i]);
			}
			else
			{
				cout << xTurtles[i].name << "Not in the way: " << xTurtles[i].position.x() << "," << xTurtles[i].position.y() << ") N(" << current.position.x() << "," << current.position.y() << ") T(" << position.x() << "," << position.y() << ")" << max(dot1, dot2) << " threshold " << angleThreshold << endl;
			}
			//cout << "\t" << xTurtles[i].name << "\n\tLHS: " << (xTurtles[i].position-current.position).normalize().x() << "," << (xTurtles[i].position-current.position).normalize().y() 
			//	<< "\n\tRHS: " << (position-current.position).normalize().x() << "," << (position-current.position).normalize().y();
			//cout << "\n\tSum: " << sum << endl;
		}
	}
	//cout << "SUM: " << sum << endl;
	hValue = getDistance(current.position.x(),current.position.y(),position.x(),position.y())*sum;
	return hValue;
}

//Replace with cost code
double Turtle::g()
{
	if(parent != NULL)
		gValue = parent->gValue;
	gValue += getDistance(parent->position.x(),parent->position.y(),position.x(),position.y());
	return gValue;
}

double Turtle::f()
{
	fValue = g() + h();
	//cout << gValue << " | " << hValue;
	return fValue;
}

void Tree::add(Turtle *parent, Turtle *node)
{
	node->parent = new Turtle();
	if(parent == NULL)
	{
		node->parent = &root;
		root.children.push_back(node);
	}else{
		parent->children.push_back(node);
		node->parent = parent;
	}
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

void Tree::setPath()
{
	if(root.children.size() == 0)
		return;
	Turtle *current = &root;
	double minF = current->children[0]->f();
	double currentF = 0;
	int minIndex = 0;


	for (int i = 0; i < root.children.size(); i++)
	{
		if(current->children.size()>0)
		{
			minF = current->children[0]->f();
		}
		minIndex = -1;
		for (int j = 0; j < current->children.size(); j++)
		{
			currentF = current->children[j]->f();
			cout << current->children[j]->name << " f()=" << currentF << endl;
			if (currentF <= minF)
			{
				minF = currentF;
				minIndex = j;
			}
		}

		for (int j = 0; j < current->children.size(); j++)
		{
			if (j != minIndex)
			{
				current->children[minIndex]->children.push_back(current->children[j]);
				current->children[j]->parent = current->children[minIndex];
			}
		}

		if(minIndex >= 0)
		{
			path.push_back(current->children[minIndex]);
			current = current->children[minIndex];
			cout << "Path " << i << ": " << current->name << " F=" << current->f() << " (" << current->position.x() << "," << current->position.y() << ")" 
			<< "From:" << current->parent->name << endl;
		}	
	}
	cout << "Done" << endl;
}

vector<Turtle *> Tree::getPath()
{
	return path;
}

void get_all_turts(const turtlesim::Pose::ConstPtr & pose_message, Tree *tree, Turtle *t)
{
	// after the initial update we don't need to worry anymore
	if (!T_turtlesHasUpdated && t->pose.x == -1 && t->pose.y == -1) 
	{
		turtlesim::Pose temp;

		temp.x = pose_message->x;
		temp.y = pose_message->y;
		temp.theta = pose_message->theta;

		t->position.set(pose_message->x,pose_message->y);
		t->pose = temp;
		
		if(t->type == "T")
		{
			tree->add(NULL,t);
			num_Tturtles_updated++;
		}
		else if(t->type == "X")
		{
			cout << "Added " << t->name << endl;
			xTurtles.push_back(*t);
			num_Xturtles_updated++;
		}
	}


	if (num_Tturtles_updated+num_Xturtles_updated >= num_Tturtles+num_Xturtles || T_turtlesHasUpdated) 
	{
		T_turtlesHasUpdated = true;
	}
}

//using hw3test.cpp as a reference
void kill_turtle(string victim_name) 
{
	turtlesim::Kill::Request reqk;
	turtlesim::Kill::Response respk;

	reqk.name = victim_name;
	if (!kClient.call(reqk, respk))
    	ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
  	else
    	ROS_INFO_STREAM("Just ate " << victim_name << "\n");
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

	//For killing turtles
	kClient = n.serviceClient<turtlesim::Kill>("kill");
	//num_turtles = alltopics.size()-1;

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
			if(i==2)
			{
				cout << topic_string_T << " VS " << alltopics[j].name << endl;
			}
			if (alltopics[j].name.compare(topic_string_T) == 0)
			{
				foundT = true;
				t = new Turtle();
				t->name = name_stream_T.str();
				t->type = "T";
				t->sub = n.subscribe<turtlesim::Pose>(topic_string_T, 10, boost::bind(get_all_turts, _1, &tree, t));
				num_Tturtles++;
			}



			if (alltopics[j].name.compare(topic_string_X) == 0 && !foundX)
			{
				foundX = true;
				t = new Turtle();
				t->name = name_stream_X.str();
				t->type = "X";
				t->sub = n.subscribe<turtlesim::Pose>(topic_string_X, 10, boost::bind(get_all_turts, _1, &tree, t));
				num_Xturtles++;
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
	tree.root.position.set(navTurtle.position.x(),navTurtle.position.y());


	std::cout << "\nPRINTING X TURTLES: " << std::endl;
	for (int i = 0; i < xTurtles.size(); i++)
	{
		//xTurtles[i].print();
		std::cout << "\nName: " << xTurtles[i].name << "\nType: " << xTurtles[i].type << "\nLocation: (" << xTurtles[i].pose.x << "," << xTurtles[i].pose.y << ")" << std::endl;
	}

	std::cout << "\nPRINTING TREE: " << std::endl;
	
	cout << tree.root.children.size() << endl;
	//tree.printat(tree.root);
	tree.setPath();

	//tree.printat(tree.root);

	//-----Move the turtle-----
	turtlesim::Pose start_pose = navTurtle.pose;
	turtlesim::Pose goal_pose;

	//start timer
	double time_started = ros::Time::now().toSec();

	for(int i = 0; i < tree.getPath().size(); i++)
	{
		cout << "Moving Around " << tree.getPath()[i]->name << endl;
		if(tree.getPath()[i]->xInTheWay.size()>0)
		{
			Tree avoid;

			cout << "0" << endl;
			Turtle dest = *tree.getPath()[i];
			cout << "1" << endl;
			//avoid.add(NULL,&dest);
			cout << "2" << endl;
			Turtle start = *tree.getPath()[(i>0?i-1:0)];
			cout << "3" << endl;
			start.parent = &dest;
			avoid.root.position.set(start.position.x(),start.position.y());
			cout << "4" << endl;

			for(int j = 0; j < tree.getPath()[i]->xInTheWay.size(); j++)
			{
				Turtle *t;

				t = new Turtle();
				cout << "5" << endl;
				t->position = Vector2(tree.getPath()[i]->xInTheWay[j].position.x()+1,tree.getPath()[i]->xInTheWay[j].position.y());
				t->type = "X";
				if((t->position.x() >= 0)&&(t->position.y() >= 0)&&(t->position.x() <= 11)&&(t->position.y() <= 11)
				&&((t->position - start.position).normalize() * (tree.getPath()[i]->position - start.position).normalize() > 0))
				{
					cout << "Adding " << t->position.x() << "," << t->position.y() << endl;
					avoid.add(NULL,t);
				}
				cout << "5" << endl;
				//parent->children.push_back(t);

				t = new Turtle();
				t->position = Vector2(tree.getPath()[i]->xInTheWay[j].position.x()-1,tree.getPath()[i]->xInTheWay[j].position.y());
				t->type = "X";
				if((t->position.x() >= 0)&&(t->position.y() >= 0)&&(t->position.x() <= 11)&&(t->position.y() <= 11)
				&&((t->position - start.position).normalize() * (tree.getPath()[i]->position - start.position).normalize() > 0))
				{
					cout << "Adding " << t->position.x() << "," << t->position.y() << endl;
					avoid.add(NULL,t);
				}
				cout << "6" << endl;
				//parent->children.push_back(t);

				t = new Turtle();
				t->position = Vector2(tree.getPath()[i]->xInTheWay[j].position.x(),tree.getPath()[i]->xInTheWay[j].position.y()+1);
				t->type = "X";
				if((t->position.x() >= 0)&&(t->position.y() >= 0)&&(t->position.x() <= 11)&&(t->position.y() <= 11)
				&&((t->position - start.position).normalize() * (tree.getPath()[i]->position - start.position).normalize() > 0))
				{
					cout << "Adding " << t->position.x() << "," << t->position.y() << endl;
					avoid.add(NULL,t);
				}
				//parent->children.push_back(t);

				t = new Turtle();
				t->position = Vector2(tree.getPath()[i]->xInTheWay[j].position.x(),tree.getPath()[i]->xInTheWay[j].position.y()-1);
				t->type = "X";
				if((t->position.x() >= 0)&&(t->position.y() >= 0)&&(t->position.x() <= 11)&&(t->position.y() <= 11)
				&&((t->position - start.position).normalize() * (tree.getPath()[i]->position - start.position).normalize() > 0))
				{
					cout << "Adding " << t->position.x() << "," << t->position.y() << endl;
					avoid.add(NULL,t);
				}
				//parent->children.push_back(t);
			}

			cout << "Intermidate Path count: " << avoid.root.children.size() << endl;
			avoid.setPath();

			for(int j = 0; j < avoid.getPath().size(); j++)
			{
				dest.parent = &navTurtle;//avoid.getPath()[j>0?j-1:0];
				cout << "Moving to inter " << j << endl;
				if(dest.h() == 0)
				{
					cout << "Path Clear! Moving to destination" << endl;
					break;
				}

				//If Moving towards the goal
				if((avoid.getPath()[j]->position - dest.parent->position) * (dest.position - dest.parent->position) > 0
					&& avoid.getPath()[j]->hValue == 0)
				{
					goal_pose.x = avoid.getPath()[j]->position.x();
					goal_pose.y = avoid.getPath()[j]->position.y();
					move_goal(goal_pose);
				}
				
			}

		}
		goal_pose.x = tree.getPath()[i]->position.x();
		goal_pose.y = tree.getPath()[i]->position.y();
		move_goal(goal_pose);

		kill_turtle(tree.getPath()[i]->name);
		//tree.getPath()[i]->kill();
	}
	//end timer
	double time_ended = ros::Time::now().toSec();

	//Total time it took
	double time_it_took = time_ended - time_started;
	cout << "Time it took: " << time_it_took << " seconds" << endl;
	cout << "Time it took walking: " << time_it_took_walking << " seconds" << endl;

	//Total Distance traveled
	cout << "Total distance traveled: " << total_distance << endl; 

	//Average velocity (counting in time turning)
	cout << "Average Velocity (distance per second) (counting in time taken turning): " << (total_distance / time_it_took) << endl;

	//Average velocity (not counting in time turning)
	cout << "Average Velocity (distance per second) (only counting in time walking): " << (total_distance / time_it_took_walking) << endl;

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
	double start_walk_time, end_walk_time; //For tracking time it took walking (for avg velocity only counting walking)

	//For calculating total distance (assuming we only use move_goal() to move)
	total_distance += getDistance(navTurtle.pose.x, navTurtle.pose.y, goal_pose.x, goal_pose.y);

	//rotate the turtle first
	do 
	{
		//The closer we are pointed in the direction of the 'goal_pose', angular velocity goes closer to 0
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 1.0 * (atan2(goal_pose.y-navTurtle.pose.y, goal_pose.x - navTurtle.pose.x) - navTurtle.pose.theta);

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

		
		//chosen tolerance for angular velocity is 0.001
	} while (vel_msg.angular.z >= 0.001 || vel_msg.angular.z <= -0.001);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);


	start_walk_time = ros::Time::now().toSec();
	//make the turtle walk
	do 
	{
		//Once we reach the desired 'goal_pose', linear velocity goes to 0
		vel_msg.linear.x = 1.0 * getDistance (navTurtle.pose.x, navTurtle.pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

		//chosen tolerance for linear velocity is 0.05
	} while (getDistance(navTurtle.pose.x, navTurtle.pose.y, goal_pose.x, goal_pose.y) > .16);	
	end_walk_time = ros::Time::now().toSec();

	//track the time taken walking
	time_it_took_walking += end_walk_time - start_walk_time;

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
	navTurtle.pose.x = pose_message->x;
	navTurtle.pose.y = pose_message->y;
	navTurtle.pose.theta = pose_message->theta;

	navTurtle.position.set(pose_message->x,pose_message->y);
}

//-------helper functions-------
double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2), 2) + pow ((y1-y2),2));
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}
