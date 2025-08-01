// File: rrt_path_planner/include/rrt_path_planner/rrt_global_planner.h

#ifndef RRT_GLOBAL_PLANNER_H
#define RRT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <random>
#include <vector>
#include <memory>

namespace rrt_global_planner {

struct Node {
    double x, y;
    int parent_id;
    Node(double x_, double y_, int parent_) : x(x_), y(y_), parent_id(parent_) {}
};

class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    RRTGlobalPlanner();
    RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~RRTGlobalPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    ros::Publisher plan_pub_;
    ros::Publisher tree_pub_;
    ros::Publisher path_pub_;
    
    bool initialized_;
    double step_size_;
    double goal_tolerance_;
    int max_iterations_;
    double collision_check_resolution_;
    bool use_deterministic_;  // New parameter
    int planning_attempts_;   // Track planning attempts for seed variation
    
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_x_;
    std::uniform_real_distribution<> dis_y_;
    
    // Store last goal to detect when we're replanning for the same goal
    geometry_msgs::PoseStamped last_goal_;
    
    double distance(double x1, double y1, double x2, double y2);
    int getNearestNode(const std::vector<Node>& tree, double x, double y);
    Node steer(const Node& from, double to_x, double to_y);
    bool isCollisionFree(double x1, double y1, double x2, double y2);
    bool isPointValid(double x, double y);
    std::vector<geometry_msgs::PoseStamped> extractPath(const std::vector<Node>& tree, 
                                                        int goal_node_id,
                                                        const geometry_msgs::PoseStamped& start,
                                                        const geometry_msgs::PoseStamped& goal);
    void publishTree(const std::vector<Node>& tree);
    void publishPath(const std::vector<geometry_msgs::PoseStamped>& path);
    std::vector<geometry_msgs::PoseStamped> smoothPath(const std::vector<geometry_msgs::PoseStamped>& path);
    size_t generateSeed(const geometry_msgs::PoseStamped& start, 
                       const geometry_msgs::PoseStamped& goal,
                       int attempt_number);
    // // Add these methods to your RRTGlobalPlanner class
    // bool checkCornerCollision(double x1, double y1, double x2, double y2);
    // bool isCollisionFreeWithMargin(double x1, double y1, double x2, double y2, double margin);
    // bool isPointValidWithRadius(double x, double y, double radius);
    // std::vector<geometry_msgs::PoseStamped> addIntermediateWaypoints(
    // const std::vector<geometry_msgs::PoseStamped>& path);
};

} // namespace rrt_global_planner

#endif // RRT_GLOBAL_PLANNER_H


