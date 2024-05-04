#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <functional>
#include "rp_loc/dmap_localizer.h"
#include "rp_base/grid_map.h"


double map_resolution;
double map_width;
double map_height;
std::vector<double> dmap_vector;
nav_msgs::OccupancyGrid::ConstPtr global_map;
bool initial_pose_received = false, goal_pose_received = false;

///////////////// Definition of useful structures///////////////
struct Pose {
    double x;
    double y;
};

struct Node {
    Pose pose;
    double cost;
    Node* parent;
    Node(Pose p, double c, Node* par) : pose(p), cost(c), parent(par) {}
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};


//////////////////////////Utility functions///////////////////////
Pose worldToGrid(const Pose& world_pose) {
    Pose grid_pose;
    grid_pose.x = std::round((world_pose.x - global_map->info.origin.position.x) / map_resolution);
    grid_pose.y = std::round((world_pose.y - global_map->info.origin.position.y) / map_resolution);
    return grid_pose;
}

Pose gridToWorld(const Pose& grid_pose) {
    Pose world_pose;
    world_pose.x = global_map->info.origin.position.x + grid_pose.x * map_resolution;
    world_pose.y = global_map->info.origin.position.y + grid_pose.y * map_resolution;
    return world_pose;
}

/////////////////////////Distance map//////////////////////////
std::vector<double> distance_map(string filename, float resolution, float dmax) {
  //Load the map image and calculate the grid map of the distances
  GridMap grid_map(0,0,resolution);
  grid_map.loadFromImage(filename.c_str(), resolution);
  DMapLocalizer localizer;
  localizer.setMap(grid_map, dmax);
  const auto& distances = localizer.distances;

  //Extract and return the dmap_vector
  vector<double> dmap_vector;
  for (size_t i=0; i<distances.cells.size(); ++i) {
    dmap_vector.push_back(distances.cells[i]);
  }
  std::reverse(dmap_vector.begin(), dmap_vector.end());  
  return dmap_vector;
}




//////////////////////////Cost function//////////////////////////
double cost_function(double current_x, double current_y,double constant) {

    // Calculate the index and acces the element in the dmap_vector
    int index = current_x * map_width + current_y;
    double element = dmap_vector[index];

    // Normalize the element value
    element = element / 255.0;
    
    // Calculate exponential cost based on the element with a costant to adjust the result
    double cost = exp(-constant * element);
    
    // Return the calculated cost
    return cost;
}


////////////////////////////////UCS//////////////////////////////////
std::vector<geometry_msgs::PoseStamped> uniformCostSearch(const Pose& start, const Pose& goal,double constant) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
    std::unordered_map<int, bool> visited;

    Pose grid_start = worldToGrid(start);
    Pose grid_goal = worldToGrid(goal);

    frontier.push(Node(grid_start, 0.0, nullptr));

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        int idx = current.pose.y * map_width + current.pose.x;

        if (visited[idx]) {
            continue;
        }
        visited[idx] = true;


        if (current.pose.x == grid_goal.x && current.pose.y == grid_goal.y) {
            // Backtrack to find the path if you find the goal
            std::vector<geometry_msgs::PoseStamped> path;
            for (Node* n = &current; n != nullptr; n = n->parent) {
                geometry_msgs::PoseStamped poseStamped;
                Pose world_pose = gridToWorld(n->pose);
                poseStamped.pose.position.x = world_pose.x;
                poseStamped.pose.position.y = world_pose.y;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.header.frame_id = "map";
                path.push_back(poseStamped);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors 
        std::vector<Pose> neighbors = {
            {current.pose.x + 1, current.pose.y},
            {current.pose.x - 1, current.pose.y},
            {current.pose.x, current.pose.y + 1},
            {current.pose.x, current.pose.y - 1}
        };        
        for (const auto& neighbor : neighbors) {
        int NewIdx = neighbor.y * map_width + neighbor.x;
            if (!visited[NewIdx]&& global_map->data[NewIdx] == 0) {
                double cost = current.cost + cost_function(neighbor.x, neighbor.y,constant);
                frontier.push(Node(neighbor, cost, new Node(current.pose, current.cost, current.parent)));
            }
        }
    }

    return {}; // return empty path if no path found
}






////////////////////////////////Callbacks////////////////////////////
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    global_map = map;
    map_width=global_map->info.width;
    map_height=global_map->info.height;
    map_resolution = map->info.resolution;
    ROS_INFO("Received Map from map_server: resolution= %lf,width=%lf,height=%lf", map_resolution,map_width,map_height);
}
Pose initial_pose, goal_pose;
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    initial_pose.x = msg->pose.pose.position.x;
    initial_pose.y = msg->pose.pose.position.y;
    initial_pose_received = true;
    ROS_INFO("Received initial pose: x=%f, y=%f", initial_pose.x, initial_pose.y);
}

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    goal_pose_received = true;
    ROS_INFO("Received goal pose: x=%f, y=%f", goal_pose.x, goal_pose.y);

}





///////////////////////Main///////////////////////////////
   int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node");
    
    if (argc < 4) {
    ROS_ERROR("User must enter 3 arguments:dmax,constant and MapID!");
    return -1;
}
    // Parse user arguments
    double dmax = atof(argv[1]);
    double constant=atof(argv[2]);
    int MapID=atof(argv[3]);
 
    // NodeHandle and Subscriber setup
    ros::NodeHandle nh;
    ros::Subscriber initial_pose_subscriber = nh.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber goal_pose_subscriber = nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    ros::Subscriber map_subscriber = nh.subscribe("/map", 1, mapCallback);
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/path", 1);
    ros::Rate rate(1); 

    // Wait for initial and goal poses
    while ((!initial_pose_received || !goal_pose_received) && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for initial and goal poses...");
    }

    // Wait for the map
    while (!global_map && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for map...");
    }

    // Load and process the distance map
    std::vector<geometry_msgs::PoseStamped> path;
    try {
        if(MapID==1){
         dmap_vector = distance_map("/home/lattinone/RP-Simple_Planner/catkin_ws/src/ros_node/src/DIAG_map.png", map_resolution, dmax);
        }else{
        dmap_vector = distance_map("/home/lattinone/RP-Simple_Planner/catkin_ws/src/ros_node/src/maze.png", map_resolution, dmax);
        }
       path = uniformCostSearch(initial_pose, goal_pose,constant);
     } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
       if (! path.empty()){
          ROS_INFO("PATH FOUND, number of poses: %ld", path.size());

        // Publish and log the path
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = path;
        path_publisher.publish(path_msg);

        for (const auto& pose : path) {
            ROS_INFO("Path Pose: x=%f, y=%f", pose.pose.position.x, pose.pose.position.y);
        }
       }else{
        ROS_WARN("NO PATH FOUND");
       }

    ros::spin();
    return 0;
}

