# CPSC481 HW2

<br>

## Steps to Run:
1. Create a package folder “hw2” in a ros work space, under the “src” folder
2. Copy "CMakeLists.txt", "package,xml" into hw2 package folder
3. Create a folder “src” under the hw2 folder, and copy  "shape1.cpp", "shape2.cpp" into the src folder
4. Run "catkin_make" from the root of work space

### To run the “shapeX.cpp” (“X” represent 1 or 2):
terminal 1: 
	- Run the master node
```
roscore
```

terminal 2: Run turtlesim
```
source devel/setup.bash
rosmake turtlesim
rosrun turtlesim turtlesim_node
```

terminal 3: Run shapeX
```
catkin_make
source devel/setup.bash
rosrun hw2 shapeX
```
README






