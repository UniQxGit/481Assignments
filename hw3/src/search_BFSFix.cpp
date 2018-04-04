# include <iostream>
# include <string>
# include <vector>
# include <cmath>
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
	Point getPos();

private:
	Point pos;
	string name;
	char type;
};

// global variables
vector<Turtle> remainingTturtle;// = {Turtle("T1", 8, 8), Turtle("T2", 9, 9), Turtle("T3", 10, 7), Turtle("T4", 11, 8), Turtle("T5", 9, 10) }; // value given for test. should take input from John's code
vector<Turtle> Xturtle;// = { Turtle("X1", 3, 6), Turtle("X2", 10, 10), Turtle("X3", 5, 7), Turtle("X4", 9, 7) }; // value given for test. should take input from John's code
vector<Point> nextSteps;
bool searchSpaceMatrix[23][23];
double heuristicMatrix[23][23];
Turtle* TturtleMatrix[23][23];

vector<Point> path;
vector<Turtle> TturtleEncoutered;

//Debugging:
Point lastStep(-1,-1);
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

double sumDistanceToAllRemainTturtle (Point curr){
	double sum = 0.0;
	for (int i = 0; i < remainingTturtle.size(); i++) {
		sum += (sqrt((curr.x() - remainingTturtle[i].x()) * (curr.x() - remainingTturtle[i].x()) + (curr.y() - remainingTturtle[i].y()) * (curr.y() - remainingTturtle[i].y())));
	}
	return sum;
}

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
			heuristicMatrix[i][j] = distanceP2P(current, temp) + sumDistanceToAllRemainTturtle(temp) + sumXturtlePenulty(temp);//- multiple Tturtle bounus 
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
		cout << "Current: (" << current.x() << "," << current.y() << ")" << endl;
		cout << "Getting Next Steps" << endl;
		for (int i = 0; i < nextSteps.size(); i++) {
			cout << "min_nextStep: " << floor(min_nextStep) << endl;
			cout << "Next Step " << i << ": (" << nextSteps[i].x() << "," << nextSteps[i].y() << "): " << floor(heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()]) << endl; 
			if (heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()] < min_nextStep)
			{
				min_nextStep = heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()];
				index = i;
			}
		}
		
		cout << "Setting Least step to " << "nextSteps[" << index << "] : (" << nextSteps[index].x() << "," << nextSteps[index].y() << ")" << endl;
		cout << "Last Step (" << lastStep.x() << "," << lastStep.y() << ")" << endl;

		if(nextSteps[index].x() == lastStep.x() && nextSteps[index].y() == lastStep.y())
		{
			cout << "Looping! This space has already been visited" << endl;
			return true;
		}

		if(nextSteps.size() == 0)
		{
			cout << "Stuck! Nowhere to go!" << endl;
		}
		lastStep = current;
		current.setXY(nextSteps[index].x(), nextSteps[index].y());
		path.push_back(nextSteps[index]);
		 
		if(nextSteps[index].x() == -1 || nextSteps[index].x() > 22 || nextSteps[index].y() == -1 || nextSteps[index].y() > 22)
		{
			cout << "Out of bounds!" << endl;
			return true;
		}

		// if incounters Tturtle, put in a path list, and remove from remainingTturtle
		for (int i = 0; i < remainingTturtle.size(); i++) {
			if (distanceP2P(current, remainingTturtle[i].getPos()) <= 0.5) {
				cout << "Found Turtle (" << remainingTturtle[i].x() << "," << remainingTturtle[i].y() << ")" << endl;
				TturtleEncoutered.push_back(remainingTturtle[i]);
				remainingTturtle.erase(remainingTturtle.begin() + i);  // update remaining T turtle vector
				setTturtles();  // update remaining T turtles matrix
			}
		}

		cout << "\n" << endl;
		searchAllTturtle(current);
	}
}

int main() {
	//c++98 compliance
	remainingTturtle.push_back(Turtle("T1", 8, 8));
	remainingTturtle.push_back(Turtle("T2", 9, 9));
	remainingTturtle.push_back(Turtle("T3", 10, 7));
	remainingTturtle.push_back(Turtle("T4", 11, 8));
	remainingTturtle.push_back(Turtle("T5", 9, 10));

	Xturtle.push_back(Turtle("X1", 3, 6));
	Xturtle.push_back(Turtle("X2", 10, 10));
	Xturtle.push_back(Turtle("X3", 5, 7));
	Xturtle.push_back(Turtle("X4", 9, 7));

	setTturtles(); // this function takes input from hw3.cpp -> works!

	setSearchSpace();   // -> works!

	int INIX = 6, INIY = 6; // double INIX=5.544445, INIY=5.544445, will take input from hw3test.cpp 
	Point spawnPoint (INIX * 2, INIY * 2);


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

	remainingTturtle.push_back(Turtle("T1", 8, 8));
	remainingTturtle.push_back(Turtle("T2", 9, 9));
	remainingTturtle.push_back(Turtle("T3", 10, 7));
	remainingTturtle.push_back(Turtle("T4", 11, 8));
	remainingTturtle.push_back(Turtle("T5", 9, 10));
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
			if(TturtleMatrix[i][j] != NULL)
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
	return 0;
}
