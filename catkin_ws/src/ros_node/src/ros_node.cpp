#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <limits> 
#include "rp_loc/dmap_localizer.h"
#include "rp_base/grid_map.h"

double map_resolution;
double map_width;
double map_height;
std::vector<double> dmap_vector;
nav_msgs::OccupancyGrid::ConstPtr global_map;

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
    grid_pose.x = (world_x - global_map->info.origin.position.x) / map_resolution;
    grid_pose.y = (world_y - global_map->info.origin.position.y) / map_resolution;
    return grid_pose;
}

Pose gridToWorld(const Pose& grid_pose) {
    Pose world_pose;
    world_pose.x = global_map->info.origin.position.x + grid_x * map_resolution;
    world_pose.y = global_map->info.origin.position.y + grid_y * map_resolution;
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






///////////////////////Main///////////////////////////////
   int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node");

    double dmax = atof(argv[1]);
    double x = atof(argv[2]);
    double y = atof(argv[3]);

    ros::NodeHandle nh;
    ros::Subscriber map_subscriber = nh.subscribe("/map", 1, mapCallback);
    ros::Rate rate(1); 

    
    // Wait for the map to be received
    ros::Time start_time = ros::Time::now();
    while (!global_map && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        if ((ros::Time::now() - start_time) > ros::Duration(10.0)) {
            ROS_ERROR("Timeout while waiting for the map.");
            return -1;
        }
    }

    if (!global_map) {
        ROS_ERROR("No map received.");
        return -1;
    }

    try {
        dmap_vector = distance_map("/home/lattinone/RP-Simple_Planner/catkin_ws/src/ros_node/src/DIAG_map.png", map_resolution, dmax);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception caught while loading or processing the map: %s", e.what());
        return -1;
    }

    // Continue to process and report cost
    while (ros::ok()) {
        if (!dmap_vector.empty()) {
            try {
                double cost = cost_function(x, y);
                ROS_INFO("The cost at position x=%lf and y=%lf is cost=%lf", x, y, cost);
            } catch (const std::exception& e) {
                ROS_ERROR("Exception caught while computing cost: %s", e.what());
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}