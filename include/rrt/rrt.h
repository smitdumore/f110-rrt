// ESE 680
// RRT assignment
// Author: Hongrui Zheng

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


struct Node {
    Node() = default;
    Node(const double x, const double y, const int parent_index) :
        x(x), y(y), parent_index(parent_index)
    {}

    double x, y;
    int parent_index; // index of parent node in the tree vector   
};


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // subs
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;

    //pubs
    ros::Publisher dynamic_map_pub_;
    ros::Publisher point_pub_;
    ros::Publisher line_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher local_point_pub_;
    ros::Publisher drive_pub_;


    // tf stuff
    tf::TransformListener listener;

    //csv
    std::vector<std::array<double, 2>> global_path_;         //array of double with size 2            vector< array(x, y) >
    std::vector<std::array<double, 2>> local_path_;

    // TODO: create RRT params

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    //transformations
    /*
    Here, we create a TransformListener object.
     Once the listener is created, 
     it starts receiving tf2 transformations over the wire, 
    and buffers them for up to 10 seconds. 
    The TransformListener object should be scoped to persist otherwise its cache will be unable to fill and almost every query will fail. 
    A common method is to make the TransformListener object a member variable of a class. 
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

    //drive params
    double high_speed_;
    double medium_speed_;
    double low_speed_;

    // Pose (Map Frame)
    double current_x_;
    double current_y_;

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    //
    std::vector<int> get_expanded_row_major_indices(const double, const double);
    //
    int xy_to_grid(std::pair<double, double> );
    //
    std::pair<double, double> grid_to_xy(int );
    //
    std::array<double, 2> get_best_global_trackpoint(const std::array<double, 2>& );
    //
    int get_row_major_index(const double, const double);
    //
    bool is_collided(const double, const double);
    //
    bool is_edge_collided(const Node &nearest_node, const Node &new_node);
    //
    void publish_corrected_speed_and_steering(double );

    // RRT methods
    std::array<double, 2> sample();
    //
    int nearest(std::vector<Node> &tree, std::array<double,2> &sampled_point);
    //
    Node steer(Node &nearest_node,const int ,std::array<double, 2> &sampled_point);
    //
    std::pair<std::array<double, 2>, double> get_best_local_trackpoint(const std::array<double, 2> &);
    
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<std::array<double ,2>> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

    int count=0;

    void init_point(visualization_msgs::Marker* point_msg){
        
        point_msg->header.frame_id = "map";
        point_msg->id = 0; //count++
        point_msg->type = visualization_msgs::Marker::SPHERE;
        point_msg->action = visualization_msgs::Marker::ADD;
        point_msg->pose.orientation.w = 1.0;
        
    }

    void pub_point(visualization_msgs::Marker* point_msg, ros::Publisher* point_pub, 
                double x, double y, bool trackpoint){

        point_msg->header.stamp = ros::Time::now();

        point_msg->pose.position.x = x;
        point_msg->pose.position.y = y;
        point_msg->pose.position.z = 0.0;

        point_msg->scale.x = 0.08;
        point_msg->scale.y = 0.08;
        point_msg->scale.z = 0.08;

        point_msg->color.a = 1.0;
        point_msg->color.r = 1.0;
        if(trackpoint){
            point_msg->scale.x = 0.2;
            point_msg->scale.y = 0.2;
            point_msg->scale.z = 0.2;
            point_msg->color.r = 0.0;
            point_msg->color.g = 1.0;
        }
        point_msg->color.b = 0.0;

    // Publish point_msg
        point_pub->publish(*point_msg);
        

    }

    void init_line(visualization_msgs::Marker* line_msg, double scale){
    
        line_msg->header.frame_id = "map";
        line_msg->id = count++;
        line_msg->type = visualization_msgs::Marker::LINE_LIST;
        line_msg->action = visualization_msgs::Marker::ADD;
        line_msg->pose.orientation.w = 1.0;
        line_msg->scale.x = scale;
    }

    void pub_line(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub, 
                double x1, double y1, double x2, double y2){

    line_msg->header.stamp = ros::Time::now();

    geometry_msgs::Point p1,p2;
    std_msgs::ColorRGBA c1 , c2;

    p1.x = x1;
    p1.y = y1;
    p1.z = 0.0;

    p2.x = x2;
    p2.y = y2;
    p2.z = 0.0;

    c1.r = 0.0;  // 1.0=255
    c1.g = 0.0;
    c1.b = 1.0;
    c1.a = 1.0;  // alpha

    c2.r = 0.0;  // 1.0=255
    c2.g = 0.0;
    c2.b = 1.0;
    c2.a = 1.0;  // alpha

    line_msg->points.push_back(p1);
    line_msg->points.push_back(p2);

    line_msg->colors.push_back(c1);
    line_msg->colors.push_back(c2);

    // Publish line_msg
    line_pub->publish(*line_msg);

    }

    
};

