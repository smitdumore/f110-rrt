#include "rrt/rrt.h"
#include "rrt/csv_reader.h"
#include <thread>
#include <chrono>


// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) , tf2_listener_(tf_buffer_){

    std::string pose_topic, scan_topic, CSV_path;
    //nh_.getParam("/pose_topic", pose_topic);
    //nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("inflation_radius", inflation_radius_);
    nh_.getParam("CSV_path", CSV_path);
    nh_.getParam("max_rrt_iters", max_rrt_iters_);
    nh_.getParam("goal_tolerance", goal_tolerance_);
    nh_.getParam("max_expansion_distance", max_expansion_distance_);
    nh_.getParam("collision_checking_points", collision_checking_points_);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance_);
    nh_.getParam("high_speed", high_speed_);
    nh_.getParam("medium_speed", medium_speed_);
    nh_.getParam("low_speed", low_speed_);

    // get the map
    input_map_ = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2.0)));

    if(input_map_.data.empty()){
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }

    ROS_INFO("Map loaded succesfully");

    map_origin_x = input_map_.info.origin.position.x;
    map_origin_y = input_map_.info.origin.position.y;
    map_cols_ = input_map_.info.width;
    map_resolution_ = input_map_.info.resolution; 
    
    //read csv
    rrt::CSVReader reader(CSV_path);
    global_path_ = reader.getData();

    //get laser to map transform
    try{
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map","laser",ros::Time(0));
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }  

    // ROS pub and sub
    pf_sub_ = nh_.subscribe("gt_pose", 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe("scan", 10, &RRT::scan_callback, this);
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map",1); 
    line_pub_ = nh_.advertise<visualization_msgs::Marker>("show_rrt_path",1); 
    waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("show_global_waypoints",1);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);

    //map
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;
    
    ROS_INFO("Created new RRT Object.");
}

int RRT::xy_to_grid(std::pair<double, double> xy_pair){
    double x = xy_pair.first;
    double y = xy_pair.second;

    auto temp_x = static_cast<int>((x - map_origin_x)/map_resolution_);
    auto temp_y = static_cast<int>((y - map_origin_y)/map_resolution_);
      
    return temp_y*map_cols_ + temp_x;
}

int RRT::get_row_major_index(const double x_map, const double y_map){
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    return y_index*map_cols_ + x_index; 
}

// std::pair<double, double> RRT::grid_to_xy(int index_in_map){
//     std::pair<double, double> xy_pair;
//     double x = (index_in_map % map_cols_)*map_resolution_ + map_origin_x;
//     double y = (index_in_map / map_cols_)*map_resolution_ + map_origin_y;
//     return std::make_pair(x,y);
// }

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
   
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }
    const auto translation = tf_laser_to_map_.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map_.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msg->ranges.size()/6); 
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*start;

    for(int i=start; i < end; i++){

        theta += angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > lookahead_distance_ || y_base_link > lookahead_distance_) continue;

        //transform to map
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        //const int map_idx = xy_to_grid(std::make_pair(x_map, y_map));

        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);

        for(const auto& index: index_of_expanded_obstacles)
        {
            //if(input_map_.data[index]  50)
            //{
                input_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            //}
        }
    }

    // local cost map clearing 
    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 10){
        for(const auto index: new_obstacles_){
            input_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        //ROS_INFO("Obstacles Cleared");
    }
         

    dynamic_map_pub_.publish(input_map_);
    //ROS_INFO("Map Updated");
    
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    
    current_x_ = pose_msg->pose.position.x;
    current_y_ = pose_msg->pose.position.y;

    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

    //publishing this point
    viz_point(trackpoint);

    std::vector<Node> tree;

    tree.emplace_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, -1));    

    int count=0;
    while(count < max_rrt_iters_){
        count++;
        
        //sample a node
        auto sample_node = sample();

        // check collision
        if(is_collided(sample_node[0], sample_node[1])){
            continue;
        }
        
        //get nearest node in the tree
        const int nearest_node_id = nearest(tree, sample_node); 

        //add current node to the tree
        Node new_node = ConstructTree(tree[nearest_node_id], nearest_node_id, sample_node);

        const auto current_node_index = tree.size();

        //check if edge is collision free
        if(is_edge_collided(tree[nearest_node_id], new_node))
        {
            continue;
        }

        tree.emplace_back(new_node);

        //check if new node is the goal
        if(is_goal(new_node ,trackpoint[0], trackpoint[1])){
            ROS_WARN("goal reached, backtracking");

            // local path points are in map frame
            local_path_ = find_path(tree, new_node);
            
            viz_path(local_path_);

            const auto trackpoint_and_distance =
                    get_best_local_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

            const auto local_trackpoint_map_frame = trackpoint_and_distance.first;
            const double distance = trackpoint_and_distance.second;

            geometry_msgs::Pose goal_way_point_car_frame;

            tf2::doTransform(local_trackpoint_map_frame, goal_way_point_car_frame, tf_map_to_laser_);

            const double steering_angle = 2*(goal_way_point_car_frame.position.y)/pow(distance, 2);

            publish_corrected_speed_and_steering(steering_angle);

            break;

        }
        
    }

}

std::array<double ,2> RRT::sample() {

    std::uniform_real_distribution<>::param_type x_param(0, lookahead_distance_);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_distance_, lookahead_distance_);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;
    tf2::doTransform(sample_point, sample_point, tf_laser_to_map_);


    // Sampled points are returned in map frame
    return {sample_point.position.x, sample_point.position.y};
}


int RRT::nearest(std::vector<Node> &tree, std::array<double,2> &sampled_point) {

    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();

    for(int i=0; i < tree.size(); i++){
        const auto dist = pow( pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2) ,0.5);
        if(dist < nearest_node_distance){
            nearest_node = i;
            nearest_node_distance = dist;
        }
    }

    return nearest_node;
}

Node RRT::ConstructTree(Node &nearest_node, const int nearest_node_index, std::array<double, 2> &sampled_point) {
    
    const double x_diff = sampled_point[0] - nearest_node.x;
    const double y_diff = sampled_point[1] - nearest_node.y;
    const double distance = pow(pow(x_diff, 2) + pow(y_diff, 2), 0.5);

    Node new_node{};
    if(distance < max_expansion_distance_){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else{
        const double theta = atan2(y_diff, x_diff);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2)+pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance_;
}

std::vector<std::array<double , 2>> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    
    std::vector<std::array<double ,2>> path;
    Node current_node = latest_added_node;

    while(current_node.parent_index != -1){
        std::array<double ,2> node_xy{current_node.x, current_node.y};
        path.emplace_back(node_xy);
        current_node = tree[current_node.parent_index];
    }
    return path;
}

std::pair<geometry_msgs::Pose, double> RRT::get_best_local_trackpoint(const std::array<double, 2> &current_pose){

    geometry_msgs::Pose closest_point{};
    double closest_distance_to_current_pose =std::numeric_limits<double>::max();
    double closest_distance = std::numeric_limits<double>::max();

    for(const auto& itr : local_path_){
        double dist = sqrt(pow(itr[0] - current_pose[0], 2)
                               + pow(itr[1] - current_pose[1], 2));

        double diff_distance = std::abs(local_lookahead_distance_ - dist);
        if(diff_distance < closest_distance)
        {
            closest_distance_to_current_pose = dist;
            closest_distance = diff_distance;
            closest_point.position.x = itr[0];
            closest_point.position.y = itr[1];
        }
    }    
    return {closest_point, closest_distance_to_current_pose};
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

std::vector<int> RRT::get_expanded_row_major_indices(const double x_map, const double y_map){

    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);

    for(int i=-inflation_radius_ + x_index; i< inflation_radius_ +1 + x_index; i++){

        for(int j=-inflation_radius_ + y_index; j< inflation_radius_ +1 + y_index ; j++){

            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }

    return expanded_row_major_indices;
}

std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose){

    try{
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for(int i=0; i<global_path_.size(); ++i){
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = global_path_[i][0];
        goal_way_point.position.y = global_path_[i][1];
        goal_way_point.position.z = 0.0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

        // making sure waypoint is in front of the car
        if(goal_way_point.position.x < 0) continue;

        double distance = std::abs(lookahead_distance_ -
                sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2)));

        if(distance < best_trackpoint_distance){
            const auto row_major_index = get_row_major_index(global_path_[i][0], global_path_[i][1]);
            if (input_map_.data[row_major_index] == 100) continue;
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    
    return global_path_[best_trackpoint_index];
}

bool RRT::is_collided(const double x_map, const double y_map)
{
    const auto index = get_row_major_index(x_map, y_map);
    return input_map_.data[index] == 100;
}

bool RRT::is_edge_collided(const Node &nearest_node, const Node &new_node){
    
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;
    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(is_collided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}


void RRT::publish_corrected_speed_and_steering(double steering_angle)
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "base_link";

    drive_msg.drive.steering_angle = steering_angle;
    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle > 0.4)
            {
                drive_msg.drive.steering_angle = 0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle < -0.4)
            {
                drive_msg.drive.steering_angle = -0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else
    {
        drive_msg.drive.speed = high_speed_;
    }

    drive_pub_.publish(drive_msg);
}