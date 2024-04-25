#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <limits> 
#include "rp_loc/dmap_localizer.h"
#include "rp_base/grid_map.h"

double map_resolution;
double map_width;
double map_height;
std::vector<double> dmap_vector;
nav_msgs::OccupancyGrid::ConstPtr global_map;
bool initial_pose_received = false, goal_pose_received = false;

////////////// Definition of useful structures/////////////
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


/////////////////////////Utility functions//////////////////////
Pose worldToGrid(const Pose& world_pose) {
    Pose grid_pose;
    grid_pose.x = (world_pose.x - global_map->info.origin.position.x) / map_resolution;
    grid_pose.y = (world_pose.x - global_map->info.origin.position.y) / map_resolution;
    return grid_pose;
}

Pose gridToWorld(const Pose& grid_pose) {
    Pose world_pose;
    world_pose.x = global_map->info.origin.position.x + grid_pose.x * map_resolution;
    world_pose.y = global_map->info.origin.position.y + grid_pose.x * map_resolution;
    return world_pose;
}

///////////////////////Distance map/////////////////////
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




////////////////////Cost function///////////////////////
double cost_function(double current_x, double current_y) {

    // Check if indices are within the bounds of the map dimensions
    if (current_x < 0 || current_x >= map_width || current_y < 0 || current_y >= map_height) {
        // Handle out-of-bounds error, return infinite cost for out of bounds
        return std::numeric_limits<double>::infinity();
    }

    // Calculate the index and acces the element in the dmap_vector
    int index = current_x * map_width + current_y;
    double element = dmap_vector[index];

    // Normalize the element value
    element = element / 255.0;
    
    // Check for the presence of an obstacle
    if (element == 0) {
        // Return infinite cost if on an obstacle
        return std::numeric_limits<double>::infinity();
    }
    
    // Calculate exponential cost based on the element with a costant to adjust the result
    double cost = exp(-10 * element);
    
    // Return the calculated cost
    return cost;
}








////////////////////////Callbacks////////////////////////////
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

    // Parse user arguments
    double dmax = atof(argv[1]);
    double x = atof(argv[2]);
    double y = atof(argv[3]);

    // NodeHandle and Subscriber setup
    ros::NodeHandle nh;
    ros::Subscriber initial_pose_subscriber = nh.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber goal_pose_subscriber = nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    ros::Subscriber map_subscriber = nh.subscribe("/map", 1, mapCallback);
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
    try {
        dmap_vector = distance_map("/home/lattinone/RP-Simple_Planner/catkin_ws/src/ros_node/src/DIAG_map.png", map_resolution, dmax);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception caught while loading or processing the map: %s", e.what());
        return -1;
    }

    // Calculate and report cost
    try {
        double cost = cost_function(x, y);
        ROS_INFO("The cost at position x=%lf, y=%lf is cost=%lf", x, y, cost);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception caught while computing cost: %s", e.what());
    }

    ros::spin();
    return 0;
}

