// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <limits>

//spline 
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor.hpp"
#include "xtensor-interpolate/xinterpolate.hpp"
#include "xtensor/xadapt.hpp"

// This is the definition of a node for the rrt tree
struct Node {

    // constructor of node class
    Node() = default;

    // constructor of node class
    Node(const double x, const double y, const int parent_index, double cost) :
        x(x), y(y), parent_index(parent_index), cost(cost)
    {}

    // x,y coordinates of node in map frame
    double x, y;

    // index of the parent in the tree vector
    int parent_index;

    // cumulative cost of node from the root of the tree
    double cost;
};

// This class is responsible for implementing RRT
class RRT {

public:
    // constructor
    RRT(ros::NodeHandle &nh);

    // destructor
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // subs
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;

    // pubs
    ros::Publisher dynamic_map_pub_;
    ros::Publisher line_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher spline_pub_;

    // tf stuff
    tf::TransformListener listener;

    //csv
    // global path is the x,y of points over the entire track(map)
    std::vector<std::array<double, 2>> global_path_;

    // local path is the path given by rrt
    // this path connects the lookahead point to the robot with the rrt solution path
    std::vector<std::array<double, 2>> local_path_;

    // random generator
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    //transformations
    /**
    * Here, we create a TransformListener object.
    * Once the listener is created, 
    * it starts receiving tf2 transformations over the wire, 
    * and buffers them for up to 10 seconds. 
    * The TransformListener object should be scoped to persist otherwise its cache will be unable to fill and almost every query will fail. 
    * A common method is to make the TransformListener object a member variable of a class. 
    */
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped tf_laser_to_map_;
    geometry_msgs::TransformStamped tf_map_to_laser_;

    //Map
    nav_msgs::OccupancyGrid input_map_;
    int map_cols_;
    int inflation_radius_;
    double map_resolution_;
    double map_origin_x;
    double map_origin_y;
    std::vector<size_t > new_obstacles_;
    int clear_obstacles_count_;
    
    //RRT params
    int max_rrt_iters_;
    double lookahead_distance_;
    double max_expansion_distance_;
    int collision_checking_points_;
    double goal_tolerance_;
    double local_lookahead_distance_;
    bool path_found_;

    //RRT star
    double search_radius_;
    visualization_msgs::Marker tree_marker_;
    visualization_msgs::Marker node_marker_;

    //drive params
    double high_speed_;
    double medium_speed_;
    double low_speed_;

    // Pose (Map Frame)
    double current_x_;
    double current_y_;

    int val_ = 0;
    
    /**
     * @brief This function recieves latest robot pose in map frame, typically from a particle filter
     * RRT main loop is also present in this function
     * @param pointer to pose_msg 
     */
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    /**
     * @brief This function recieves latest laser scans.
     * The occupancy grid is updated here
     * @param pointer to scan_msg 
     */
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    /**
     * @brief This function takes the x,y coordinates of a map cell and 
     * returns the row major index of that cell in the map vector 
     * @return index
     */
    int get_row_major_index(const double, const double);
    
    /**
     * @brief This function inflates all the occupied indices in the map and returns all the indices of the cells
     * that have been inflated  
     * @return vector of indices 
     */
    std::vector<int> get_inflated_row_major_indices(const double, const double);
    
    /**
     * @brief This function returns the x,y cordinates of a point the the global path
     * This point is atleast one lookahed distance away from the robot
     * @return std::array<double, 2> 
     */
    std::array<double, 2> get_best_global_trackpoint(const std::array<double, 2>& );
    
    /**
     * @brief Checks if a randomly generated node is free or occupied
     * @param x,y coordinates of node in map frame
     */
    bool is_collided(const double, const double);
    
    /**
     * @brief Checks if a edge connecting two nodes is free or occupied
     * @param x,y coordinates of both nodes in map frame
     */
    bool is_edge_collided(const Node &nearest_node, const Node &new_node);
    
    /**
     * @brief This function publishes the drive messages to the robot
     * linear velocity is inversely proportional to the steering angle
     */
    void execute_control(double );

    /**
     * @brief randomly generates a point in map frame
     * @return std::array<double, 2> 
     */
    std::array<double, 2> sample();

    /**
     * @brief returns the index of node in the tree vector that is nearest to the node that is passed in the function 
     * @param tree - tree vector
     * @param sampled_point - sampled point in free space
     * @return int - index in tree vector
     */
    int nearest(std::vector<Node> &tree, std::array<double,2> &sampled_point);
    
    /**
     * @brief Expands the tree towards the sample point (within a max distance)
     * @param nearest_node - nearest node to the sampled point
     * @param sampled_point - sampled point in free space
     * @return Node - new node
     */
    Node Steer(Node &nearest_node,const int ,std::array<double, 2> &sampled_point);
    
    /**
     * @brief Get the best local trackpoint from the local path for pure pursuit following
     * This local point is atleast one local lookahead distance away from the robot 
     * @return std::pair<geometry_msgs::Pose, double> 
     */
    std::pair<geometry_msgs::Pose, double> get_best_local_trackpoint(const std::array<double, 2> &);
    
    /**
     * @brief Checks if the lastest added node is the goal node or not
     * @param latest_added_node 
     * @param goal_x - x coordinate of goal in map 
     * @param goal_y - y coordinate of goal in map
     * @return true/false
     */
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);

    /**
     * @brief Backtracks the tree from goal node to root node 
     * @param tree - rrt tree vector
     * @param latest_added_node - goal node
     * @return std::vector<std::array<double ,2>> - rrt solution path 
     */
    std::vector<std::array<double ,2>> find_path(std::vector<Node> &tree, Node &latest_added_node);

    /**
     * @brief Calcutes the cumulative cost of node from the root of the tree
     * @param tree - tree vector
     * @param node - current node 
     * @return double - calculated cost
     */
    double cost(std::vector<Node> &tree, Node &node);

    /**
     * @brief Calculates the cost between two nodes
     * @param n1 - node 1
     * @param n2 - nopde 2
     * @return double - calculated cost
     */
    double line_cost(Node &n1, Node &n2);

    /**
     * @brief Returns a vector of nodes that lie within a search radius of the current node
     * @param tree - tree vector
     * @param node - current node
     * @return std::vector<int> - vector of indices of nodes inside the radius
     */
    std::vector<int> near(const std::vector<Node> &tree, Node &node);

    /**
     * @brief Connects the current node to the least cost parent and finds a child for the current node if feasible
     * @param neigh_vec - vector of nodes that lie in the search radius of the current node
     * @param tree - tree vector
     * @param node - current node 
     */
    void rewire(std::vector<int> neigh_vec, std::vector<Node> &tree, Node &node);

    /**
     * @brief Fits a cubic spline through the rrt solution points
     * @param path - rrt solution path
     * @param discrete - number of partitions 
     * @return std::vector<std::array<double ,2>> - spline points 
     */
    std::vector<std::array<double ,2>> smooth_path(std::vector<std::array<double, 2>> path, int discrete);

    /**
     * @brief Vizualizes any point in the map frame
     * @param point - point
     * @param lookahead - boolean
     */
    void viz_point(std::array<double, 2> point, bool lookahead)
    {

        visualization_msgs::Marker point_msg;
        point_msg.header.frame_id = "map";
        point_msg.type = visualization_msgs::Marker::SPHERE;
        //point_msg.id = val_++;
        point_msg.pose.orientation.w = 1.0;
        point_msg.header.stamp = ros::Time::now();

        point_msg.pose.position.x = point[0];
        point_msg.pose.position.y = point[1];
        point_msg.pose.position.z = 0.0;
        point_msg.pose.orientation.w = 1.0;

        point_msg.scale.x = point_msg.scale.y = point_msg.scale.z = 0.025;
        if(lookahead)
        {
            point_msg.scale.x = point_msg.scale.y = point_msg.scale.z = 0.1;    
        }
        point_msg.color.a = 1.0;
        point_msg.color.r = 0.0;
        point_msg.color.b = 1.0;
        point_msg.color.g = 0.0;
        
        waypoint_pub_.publish(point_msg);
        
    }

    /**
     * @brief Visualizes the rrt solution path
     * @param local_path 
     * @param curr_pose 
     */
    void viz_path(std::vector<std::array<double, 2>> local_path , geometry_msgs::Pose curr_pose)
    {
        visualization_msgs::Marker path;

        // path marker will be placed wrt map
        // path marker points are wrt map
        path.header.frame_id = "map";
        //path.id = val_++;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.scale.x = path.scale.y = 0.03;
        path.action = visualization_msgs::Marker::ADD;
        path.color.g = 1.0;
        path.color.a = 1.0;

        geometry_msgs::Point p;
        for(int i=0 ; i < local_path.size(); i++)
        {
            
            p.x = local_path[i][0];
            p.y = local_path[i][1];
            path.points.push_back(p);
        }

        //p = curr_pose.position;
        //path.points.push_back(p);

        line_pub_.publish(path);
        
    }

    /**
     * @brief Generates line_strip visualization markers 
     * @param path 
     * @return visualization_msgs::Marker 
     */
    visualization_msgs::Marker gen_path_marker(const std::vector<std::array<double,2>> &path)
    {
        std::vector<geometry_msgs::Point> tree_points = in_order_path(path);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "current";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.points = tree_points;
        marker.scale.x = 0.05;
        marker.scale.y = 0;
        marker.scale.z = 0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.4;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        return marker;
        //return gen_markers(node_coords, 0, 1, 0);
}

    /**
     * @brief Reverses the rrt path
     * @param path - path
     * @return std::vector<geometry_msgs::Point> - reversed path 
     */
    std::vector<geometry_msgs::Point> in_order_path(std::vector<std::array<double,2>> path)
    {
        std::vector<geometry_msgs::Point> path_points;
        for (int ii = 0; ii < path.size(); ++ii)
        {
            geometry_msgs::Point curr;
            curr.x = path[ii][0];
            curr.y = path[ii][1];
            curr.z = 0.15;
            path_points.push_back(curr);
        }
        return path_points;
    }

    /**
     * @brief stores entire rrt tree in a vector
     * @param tree 
     * @return std::vector<geometry_msgs::Point> 
     */
    std::vector<geometry_msgs::Point> full_tree(const std::vector<Node> &tree)
    {
        std::vector<geometry_msgs::Point> tree_points;
        for (int i = 1; i < tree.size(); ++i)
        {
            geometry_msgs::Point one;
            one.x = tree[i].x;
            one.y = tree[i].y;
            one.z = 0.1;
            tree_points.push_back(one);
            geometry_msgs::Point two;
            two.x = tree[tree[i].parent_index].x;
            two.y = tree[tree[i].parent_index].y;
            two.z = 0.0;
            tree_points.push_back(two);
        }
        return tree_points;
    }

    /**
     * @brief Creates a visualisation of all rrt tree edges
     * @param tree - tree vector
     * @param r - red color value
     * @param g - green color value
     * @param b - blue color value
     * @return visualization_msgs::Marker - list of line markers
     */
    visualization_msgs::Marker gen_tree_marker(const std::vector<Node> &tree, float r, float g, float b)
    {
        std::vector<geometry_msgs::Point> tree_points = full_tree(tree);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "current";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.points = tree_points;
        marker.scale.x = 0.02;
        marker.scale.y = 0;
        marker.scale.z = 0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        return marker;
    }

    /**
     * @brief Creates a visualisation of all rrt tree nodes
     * @param tree - tree vector
     * @param r - red color value
     * @param g - green color value
     * @param b - blue color value
     * @return visualization_msgs::Marker - list sphere markers
     */
    visualization_msgs::Marker gen_node_marker(const std::vector<Node> &tree, float r, float g, float b)
    {
        std::vector<geometry_msgs::Point> tree_points = full_tree(tree);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "current";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.z = 0.1;
        marker.points = tree_points;
        marker.scale.x = 0.045;
        marker.scale.y = 0.045;
        marker.scale.z = 0.045;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        return marker;
    }   

}; // RRT class ends


