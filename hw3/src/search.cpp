# include <iostream>
# include <vector>
using namespace std;

vector<Turtle> remainingTturtle;
vector<Turtle> Xturtle;
vector<Point> nextSteps;
bool searchSpaceMatrix[23][23];
double heuristicMatrix[23][23];

class Point {
public:
	Point() {}
	Point(int x, int y) {
		xval = x;
		yval = y;
	}
	int x() {
		return xval;
	}
	int y() {
		return yval;
	}
	void setXY(double x, double y) {
		xval = x;
		yval = y;
	}

private:
	int xval;
	int yval;
};

class Turtle {
public:
	bool isTturtle() {
		if (type == 'T')
			return true;
		else
			return false;
	}
	double x() {
		return pos.x();
	}
	double y() {
		return pos.y();
	}

private:
	Point pos;
	string name;
	char type;
};

double distanceP2P(Point a, Point b) {
	return (sqrt((a.x() - b.x()) * (a.x() - b.x()) + (a.y() - b.y()) * (a.y() - b.y())));
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
			if ((Xturtle[j].x() < curr.x() && Xturtle[j].x() > remainingTturtle[i].x()) || (Xturtle[j].x() > curr.x() && Xturtle[j].x() < remainingTturtle[i].x())) {
				sum += 1;
			}
		}
	}
	for (int i = 0; i < remainingTturtle.size(); i++) {
		for (int j = 0; j < Xturtle.size(); j++) {
			if ((Xturtle[j].y() < curr.y() && Xturtle[j].y() > remainingTturtle[i].y()) || (Xturtle[j].y() > curr.y() && Xturtle[j].y() < remainingTturtle[i].y())) {
				sum += 1;
			}
		}
	}
	return sum;
}

void setHeuristicMatrix(Point current) {
	for (int i = current.x() - 1; i < current.x() + 1; i++) {
		for (int j = current.y() - 1; j < current.y() + 1; j++) {
			Point temp(i, j);
			heuristicMatrix[i][j] = distanceP2P(current, temp) + sumDistanceToAllRemainTturtle(current) + sumXturtlePenulty(current);//- multiple Tturtle bounus 
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
		int x = 2 * Xturtle[i].x();
		int y = 2 * Xturtle[i].y();
		searchSpaceMatrix[x][y] = false;
		searchSpaceMatrix[x - 1][y] = false;
		searchSpaceMatrix[x + 1][y] = false;
		searchSpaceMatrix[x][y - 1] = false;
		searchSpaceMatrix[x][y + 1] = false;
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

void search(Point current) {
	if (remainingTturtle.size() == 0)
		printPath();
	else {
		setHeuristicMatrix(current);
		setNextSteps(current);

		int i;
		double min_nextStep = heuristicMatrix[nextSteps[0].x()][nextSteps[0].y()];
		for (i = 0; i < nextSteps.size(); i++) {
			if (heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()] < min_nextStep)
				min_nextStep = heuristicMatrix[nextSteps[i].x()][nextSteps[i].y()];
		}
		// if Tturtle encountered, put in a list
		current.setXY(nextSteps[i].x(), nextSteps[i].y());
		search(current);
	}
}

int main() {
	setTturtles;
	setSearchSpace();
	Point spawnPoint (INIX * 2, INIY * 2);


	setHeuristicMatrix(spawnPoint);
	search(spawnPoint);

}





