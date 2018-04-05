# include <iostream>
# include <sstream>
# include <string>
# include <vector>
# include <cmath>

# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "turtlesim/Pose.h"
# include "boost/bind.hpp"


using namespace std;

class Point {
public:
	Point();
	Point(int x, int y);
	int x();
	int y();
	void setXY(int x, int y);

private:
	int xval;
	int yval;
};

class Turtle {
public:
	Turtle(string n, int x, int y);
	bool isTturtle();
	int x();
	int y();
	string getName();
	Point getPos();

private:
	Point pos;
	string name;
	char type;
};

// global variables
ros::Publisher velocity_publisher; 	//topic to tell turtle where to go is /turtle1/cmd_vel
ros::Subscriber pose_subscriber;   	//topic to get current turtle position is /turtle1/pose
turtlesim::Pose turtle_pose;
vector<ros::Subscriber> subs;
bool poseHasUpdated = false;		//Used to wait for initial pose data before moving
bool T_turtlesHasUpdated = false;
int num_Xturtles_updated = 0;
int num_Tturtles_updated = 0;
int num_Xturtles = 0;
int num_Tturtles = 0;

vector<Turtle> remainingTturtle; // value given for test. should take input from John's code
vector<Turtle> TturtleEncoutered;
vector<Turtle> Xturtle; // value given for test. should take input from John's code
vector<Point> nextSteps;
vector<Point> path;
bool searchSpaceMatrix[23][23]; // the search space is a 11x11 grid with 0.5 intervals 
double heuristicMatrix[23][23]; // each unit in the grid has a heuristic value 
Turtle* TturtleMatrix[23][23];  // this matrix used to flag T turtle positions


void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

//Debugging:
Point lastStep(-1,-1);
vector<Turtle> remainingTturtleTest;

// function definations
Point::Point() {}
Point::Point(int x, int y) {
	xval = x;
	yval = y;
}
int Point::x() {
	return xval;
}
int Point::y() {
	return yval;
}
void Point::setXY(int x, int y) {
	xval = x;
	yval = y;
}


Turtle::Turtle(string n, int x, int y) {
	name = n;
	pos.setXY(2 * x, 2 * y);
}
bool Turtle::isTturtle() {
	if (type == 'T')
		return true;
	else
		return false;
}
int Turtle::x() {
	return pos.x();
}
int Turtle::y() {
	return pos.y();
}

string Turtle::getName(){
	return name;
}

Point Turtle::getPos() {
	return pos;
}


double distanceP2P(Point a, Point b) {
	return (sqrt((a.x() - b.x()) * (a.x() - b.x()) + (a.y() - b.y()) * (a.y() - b.y())));
}

void setTturtles() {
	for (int i = 0; i < 23; i++) {
		for (int j = 0; j < 23; j++) {
			TturtleMatrix[i][j] = NULL; // initiate all pointers to NULL
		}
	}
	for (int k = 0; k < remainingTturtle.size(); k++) {
		int i = remainingTturtle[k].x();
		int j = remainingTturtle[k].y();
		TturtleMatrix[i][j] = & remainingTturtle[k];
	}
}

// heuristic function part 2: sum of distance to all remaining T turtles
double sumDistanceToAllRemainTturtle (Point curr){
	double sum = 0.0;
	for (int i = 0; i < remainingTturtle.size(); i++) {
		sum += (sqrt((curr.x() - remainingTturtle[i].x()) * (curr.x() - remainingTturtle[i].x()) + (curr.y() - remainingTturtle[i].y()) * (curr.y() - remainingTturtle[i].y())));
	}
	return sum;
}

// heuristic function part 3: sum of "X turtle in between" penalty
double sumXturtlePenulty(Point curr) {
	double sum = 0.0;
	for (int i = 0; i < remainingTturtle.size(); i++) {
		for (int j = 0; j < Xturtle.size(); j++) {
			if (((Xturtle[j].x() < curr.x() && Xturtle[j].x() > remainingTturtle[i].x()) || (Xturtle[j].x() > curr.x() && Xturtle[j].x() < remainingTturtle[i].x())) &&
				((Xturtle[j].y() < curr.y() && Xturtle[j].y() > remainingTturtle[i].y()) || (Xturtle[j].y() > curr.y() && Xturtle[j].y() < remainingTturtle[i].y()))) {
				sum += 1;
			}
		}
	}
	return sum;
}

void setHeuristicMatrix(Point current) {
	for (int i = current.x() - 1; i <= current.x() + 1; i++) {
		for (int j = current.y() - 1; j <= current.y() + 1; j++) {
			Point temp(i, j);
			heuristicMatrix[i][j] = distanceP2P(current, temp) + sumDistanceToAllRemainTturtle(temp) + sumXturtlePenulty(temp);//- multiple Tturtle bounus? 
		}
	}
}

void setSearchSpace() {
	for (int i = 0; i < 23; i++) {
		for (int j = 0; j < 23; j++) {
			searchSpaceMatrix[i][j] = true;
		}
	}

	for (int i = 0; i < Xturtle.size(); i++) {
		int x = Xturtle[i].x();
		int y = Xturtle[i].y();
		searchSpaceMatrix[x][y] = false;
		searchSpaceMatrix[x - 1][y] = false;
		searchSpaceMatrix[x + 1][y] = false;
		searchSpaceMatrix[x][y - 1] = false;
		searchSpaceMatrix[x][y + 1] = false;
	}

	for(int i = 0; i < path.size(); i++)
	{
		int x = path[i].x();
		int y = path[i].y();
		searchSpaceMatrix[x][y] = false;
	}
}

bool isInSearchSpace(Point pnt) {
	if (pnt.x() > 22 || pnt.y() > 22)
		return false;
	if (searchSpaceMatrix[pnt.x()][pnt.y()] == true)
		return true;
	if (searchSpaceMatrix[pnt.x()][pnt.y()] == false)
		return false;
}

void setNextSteps(Point current) {
	nextSteps.clear();

	Point left(current.x() - 1, current.y());
	Point right(current.x() + 1, current.y()); 
	Point up(current.x(), current.y() + 1); 
	Point down(current.x(), current.y() - 1);
	Point upleft(current.x() - 1, current.y() + 1); 
	Point upright(current.x() + 1, current.y() + 1);
	Point downleft(current.x() - 1, current.y() - 1);
	Point downright(current.x() + 1, current.y() - 1);

	if (isInSearchSpace(left))
		nextSteps.push_back(left);
	if (isInSearchSpace(right))
		nextSteps.push_back(right);
	if (isInSearchSpace(up))
		nextSteps.push_back(up);
	if (isInSearchSpace(down))
		nextSteps.push_back(down);
	if (isInSearchSpace(upleft))
		nextSteps.push_back(upleft);
	if (isInSearchSpace(upright))
		nextSteps.push_back(upright);
	if (isInSearchSpace(downleft))
		nextSteps.push_back(downleft);
	if (isInSearchSpace(downright))
		nextSteps.push_back(downright);
}

void printPath() {
	for (int i = 0; i < path.size(); i++) {
		cout << "(" << path[i].x() << ", " << path[i].y() << ") ";
	}
	cout << endl;
}

// best first search
bool searchAllTturtle(Point current) {
	if (remainingTturtle.size() == 0) {
		printPath();
		return true;
	}
	else {
		setSearchSpace();
		setHeuristicMatrix(current);
		setNextSteps(current);

		int index = 0;
		double min_nextStep = heuristicMatrix[nextSteps[0].x()][nextSteps[0].y()];
		// cout << "Current: (" << current.x() << "," << current.y() << ")" << endl;
		// cout << "Getting Next Steps" << endl;
		// pick the best among all possible nest steps
		for (int i = 0; i < nextSteps.size(); i++) {
			// cout << "min_nextStep: " << floor(min_nextStep) << endl;
			// cout << "Next Step " << i << ": (" << nextSteps[i].x() << "," << nextSteps[i].y() << "): " << floor(heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()]) << endl; 
			if (heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()] < min_nextStep)
			{
				min_nextStep = heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()];
				index = i;
			}
		}
		
		// cout << "Setting Least step to " << "nextSteps[" << index << "] : (" << nextSteps[index].x() << "," << nextSteps[index].y() << ")" << endl;
		// cout << "Last Step (" << lastStep.x() << "," << lastStep.y() << ")" << endl;

		if(nextSteps[index].x() == lastStep.x() && nextSteps[index].y() == lastStep.y())
		{
			//cout << "Looping! This space has already been visited" << endl;
			return false;
		}

		if(nextSteps.size() == 0)
		{
			//cout << "Stuck! Nowhere to go!" << endl;
			return false;
		}
		lastStep = current;
		current.setXY(nextSteps[index].x(), nextSteps[index].y());
		path.push_back(nextSteps[index]);
		 
		if(nextSteps[index].x() == -1 || nextSteps[index].x() > 22 || nextSteps[index].y() == -1 || nextSteps[index].y() > 22)
		{
			//cout << "Out of bounds!" << endl;
			return false;
		}

		// if incounters Tturtle, put in a path list, and remove from remainingTturtle
		for (int i = 0; i < remainingTturtle.size(); i++) {
			if (distanceP2P(current, remainingTturtle[i].getPos()) <= 0.5) {
				//cout << "Found Turtle (" << remainingTturtle[i].x() << "," << remainingTturtle[i].y() << ")" << endl;
				TturtleEncoutered.push_back(remainingTturtle[i]);
				remainingTturtle.erase(remainingTturtle.begin() + i);  // update remaining T turtle vector
				setTturtles();  // update remaining T turtles matrix
			}
		}

		//cout << "\n" << endl;
		searchAllTturtle(current);
	}
}

void get_turts(const turtlesim::Pose::ConstPtr & pose_message, string type, string name)//
{
	// after the initial update we don't need to worry anymore
	if (!T_turtlesHasUpdated) 
	{
		if(type == "T")
		{
			for(int i = 0; i < remainingTturtle.size(); i++)
			{
				if(remainingTturtle[i].getName() == name)
					return;
			}
			remainingTturtle.push_back(Turtle(name, pose_message->x, pose_message->y));
			remainingTturtleTest.push_back(Turtle(name, pose_message->x, pose_message->y));
			num_Tturtles_updated++;
			cout << name << endl;
		}
		else if(type == "X")
		{
			for(int i = 0; i < Xturtle.size(); i++)
			{
				if(Xturtle[i].getName() == name)
					return;
			}
			Xturtle.push_back(Turtle(name, pose_message->x, pose_message->y));
			num_Xturtles_updated++;
			cout << name << endl;
		}
		
	}

	
	if (num_Xturtles_updated+num_Tturtles_updated >= num_Xturtles+num_Tturtles || T_turtlesHasUpdated) 
	{
		T_turtlesHasUpdated = true;
	}
}

int main(int argc, char ** argv) {
	//c++98 compliance
	remainingTturtle.clear();
	// remainingTturtle.push_back(Turtle("T1", 5, 9));
	// remainingTturtle.push_back(Turtle("T2", 8, 4));
	// remainingTturtle.push_back(Turtle("T3", 4, 2));
	// remainingTturtle.push_back(Turtle("T4", 6, 7));
	// remainingTturtle.push_back(Turtle("T5", 3, 8));
	// remainingTturtle.push_back(Turtle("T6", 5, 7));
	// remainingTturtle.push_back(Turtle("T7", 2, 10));

	// Xturtle.push_back(Turtle("X1", 7, 6));
	// Xturtle.push_back(Turtle("X2", 7, 3));
	// Xturtle.push_back(Turtle("X3", 10, 10));
	// Xturtle.push_back(Turtle("X4", 8, 10));
	// Xturtle.push_back(Turtle("X5", 8, 6));
	// Xturtle.push_back(Turtle("X6", 11, 7));
	// Xturtle.push_back(Turtle("X7", 2, 6));
	// Xturtle.push_back(Turtle("X8", 3, 6));
	// Xturtle.push_back(Turtle("X9", 11, 8));
	// Xturtle.push_back(Turtle("X10", 3, 10));
	
	ros::init(argc, argv, "hw3search");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	ros::master::V_TopicInfo alltopics;
	ros::master::getTopics(alltopics);

	//alltopics.size()-1;
	
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
			if (alltopics[j].name.compare(topic_string_T) == 0 && !foundT)
			{
				cout << topic_string_T << endl;
				string type = "T";
				string name = name_stream_T.str();
				subs.push_back(n.subscribe<turtlesim::Pose>(topic_string_T, 10, boost::bind(get_turts, _1, type, name)));//
				num_Tturtles++;
			}

			if (alltopics[j].name.compare(topic_string_X) == 0 && !foundX)
			{
				cout << topic_string_X << endl;
				string type = "X";
				string name = name_stream_X.str();
				subs.push_back(n.subscribe<turtlesim::Pose>(topic_string_X, 10, boost::bind(get_turts, _1, type, name)));//
				num_Xturtles++;
			}	
		}
	}

	//wait for initial 'turtle position' data to come in (absolutely necessary!)
	while(!poseHasUpdated || !T_turtlesHasUpdated)
	{
		ros::spinOnce();
	}

	cout << "Turtle Count " << remainingTturtle.size() << endl;

	int INIX = 6, INIY = 6; // double INIX=5.544445, INIY=5.544445, will take input from hw3test.cpp 
	Point spawnPoint (INIX * 2, INIY * 2);


	setTturtles(); // this function takes input from hw3.cpp -> works!

	setSearchSpace();   // -> works!

	for (int i = -1; i < 23; i++) {
		for (int j = -1; j < 23; j++) {
			if ( i == -1 || j == -1) //Extra row/col for line numbers
			{
				if (i % 2 == 0)
					cout << i << (i<10?"  ":" ");
				else if (j%2 == 0)
					cout << j << (j<10?"  ":" ");
				else 
					cout << "   ";
				continue;
			}

			if(spawnPoint.x() == i && spawnPoint.y() == j)
				cout << "S" << "  ";
			else if(TturtleMatrix[i][j] != NULL)
				cout << "T" << "  ";
			else if(!searchSpaceMatrix[i][j])
			{
				cout << (heuristicMatrix[i][j]>=0?"X":"-") << "  ";
			}
			else 
				cout << "-"  << "  ";
		}
		cout << endl;
	}
	cout << endl;


	setHeuristicMatrix(spawnPoint);

	searchAllTturtle(spawnPoint);


	/*******************************
	Debugging
	********************************/

	// for test only
	cout << endl;
	cout << "HEURISTIC VALUES" << endl;
	for (int i = -1; i < 23; i++) {
		for (int j = -1; j < 23; j++) {
			if ( i == -1 || j == -1) //Extra row/col for line numbers
			{
				if (i % 2 == 0)
					cout << i << (i<10?"  ":" ");
				else if (j%2 == 0)
					cout << j << (j<10?"  ":" ");
				else 
					cout << "   ";
				continue;
			}

			cout << floor(heuristicMatrix[i][j])  << (floor(heuristicMatrix[i][j])<10?"  ":" ");
		}
		cout << endl;
	}
	
	cout << endl;

	//Start Point
	heuristicMatrix[spawnPoint.x()][spawnPoint.y()] = -1;
	for (int i = 0; i < path.size(); i++)
	{
		//Path
		heuristicMatrix[path[i].x()][path[i].y()] = -(i+2);
	}

	// remainingTturtle.push_back(Turtle("T1", 5, 9));
	// remainingTturtle.push_back(Turtle("T2", 8, 4));
	// remainingTturtle.push_back(Turtle("T3", 4, 2));
	// remainingTturtle.push_back(Turtle("T4", 6, 7));
	// remainingTturtle.push_back(Turtle("T5", 3, 8));
	// remainingTturtle.push_back(Turtle("T6", 5, 7));
	// remainingTturtle.push_back(Turtle("T7", 2, 10));
	
	remainingTturtle.clear();
	cout << "Remaining T Turtles:" << remainingTturtle.size() << endl;
	remainingTturtle = remainingTturtleTest;
	cout << "Remaining T Turtles:" << remainingTturtle.size() << endl;
	setTturtles();

	cout << "TURTLE GRID" << endl;
	for (int i = -1; i < 23; i++) {
		for (int j = -1; j < 23; j++) {
			if ( i == -1 || j == -1) //Extra row/col for line numbers
			{
				if (i % 2 == 0)
					cout << i << (i<10?"  ":" ");
				else if (j%2 == 0)
					cout << j << (j<10?"  ":" ");
				else 
					cout << "   ";
				continue;
			}

			if(spawnPoint.x() == i && spawnPoint.y() == j)
				cout << "S" << "  ";
			else if(TturtleMatrix[i][j] != NULL)
				cout << "T" << "  ";
			else if(!searchSpaceMatrix[i][j])
			{
				cout << (heuristicMatrix[i][j]>=0?"X":"-") << "  ";
			}
			else 
				cout << "-"  << "  ";
		}
		cout << endl;
	}
	cout << endl;

	cout << "PATH" << endl;
	for (int i = -1; i < 23; i++) {
		for (int j = -1; j < 23; j++) {
			if ( i == -1 || j == -1) //Extra row/col for line numbers
			{
				if (i % 2 == 0)
					cout << i << (i<10?"   ":"  ");
				else if (j%2 == 0)
					cout << j << (j<10?"   ":"  ");
				else 
					cout << "    ";
				continue;
			}
			
			if(heuristicMatrix[i][j] < 0)
			{
				if(TturtleMatrix[i][j] != NULL)
					cout << "T!" << "  ";
				else
					cout << "P" << -heuristicMatrix[i][j] << (heuristicMatrix[i][j]>-10?"  ":" ");
			}
			else if(TturtleMatrix[i][j] != NULL)
				cout << "T" << "   ";
			else if(!searchSpaceMatrix[i][j])
				cout << "X" << "   ";
			else 
				cout << "-"  << "   ";
		}
		cout << endl;
	}

	/*******************************
	END Debugging
	********************************/

	//system("pause");
	loop_rate.sleep();
	ros::spin();
	return 0;
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	if(!poseHasUpdated)
		cout << "Done" << endl;
	poseHasUpdated = true;
	turtle_pose.x = pose_message->x;
	turtle_pose.y = pose_message->y;
	turtle_pose.theta = pose_message->theta;
}
