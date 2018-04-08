# CPSC481 HW3

<br>

## Steps to Run:

### If you haven't made a catkin workspace:
Create catkin workspace:
```
mkdir catkin_ws
cd catkin_ws
mkdir src -p
catkin_make
```

Note: Run this in your catkin_workspace before doing anything
```
source devel/setup.bash
```

### In your catkin workspace
1. copy paste our 'hw3' folder inside your workspace's 'src' folder
2. Update changes:
```
catkin_make
```

### Running it (Open 4 terminals)
1. terminal 1: Run the master node
```
roscore
```

2. terminal 2: Run turtlesim
```
rosmake turtlesim
rosrun turtlesim turtlesim_node
```

3. terminal 3: Run the hw3test.cpp to generate the T and X turtles
```
rosrun [package name] hw3test
```

4. terminal 4: Run hw3
```
rosrun hw3 hw3
```






