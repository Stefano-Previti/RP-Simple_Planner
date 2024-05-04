# RP-Simple_Planner   

## Description of the project
This project implements a ROS node called "my_node" that performs path planning using the Uniform Cost Search algorithm. Here's a brief description of its functionalities:
  1)Header Files: Includes necessary ROS and geometry-related header files.
  2)Global Variables: Declares global variables for map-related parameters, distance map vector, and flags for initial and goal pose reception.
  3)Structures: Defines two structures, Pose and Node, used to represent poses and nodes in the search algorithm.
  4)Utility Functions: Contains functions to convert world coordinates to grid coordinates and vice versa.
  5)Distance Map Function: Computes the distance map from an image file and stores it in a vector.
  6)Cost Function: Calculates the cost of traversing a cell based on its distance to the closest obstacle.
  7)Uniform Cost Search: Implements the Uniform Cost Search algorithm to find the shortest path from the start pose to the goal pose.
  8)Callback Functions: Defines callback functions to receive map, initial pose, and goal pose messages.
  9)Main Function: Initializes the ROS node and sets up subscribers and publishers. It waits for initial and goal poses and the map. Once received, it computes the path using the distance map and search       algorithm.If a path is found, it publishes and logs the path; otherwise, it logs a warning.
**This node is meant to be executed as part of a ROS system for robot navigation, where it plans a path based on received map data and user-defined initial and goal poses.**


##  Execution
To run the RP-Simple_Planner, navigate to the RP_Simple_Planner directory and follow these steps:

1)Start the ROS core to facilitate communication between nodes with the command **roscore**
2)Launch RVIZ to visualize the robot's environment and planned paths with the command **rosrun rviz rviz**
3)Run the map server node to provide the map data over the /map topic to RVIZ with the command  **rosrun map_server map_server file_yaml/map.yaml**
4)Navigate to the catkin_ws directory,build and run the ros_node executable with the desired parameters for dmax and constant:
     **catkin build**
     **source devel/setup.bash**
     **rosrun ros_node ros_node 0.5 1.5**
     
After launching the ros_node executable, configure RVIZ to subscribe to the relevant topics. Add a MAP component subscribing to the /map topic and a PATH component subscribing to the /path topic. Use the RVIZ user interface buttons to set the initial and goal poses, and the algorithm will compute the path. Once the algorithm completes, the planned path will be displayed in RVIZ.
