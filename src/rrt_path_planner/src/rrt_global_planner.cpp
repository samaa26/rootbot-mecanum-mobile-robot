#include <rrt_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(rrt_global_planner::RRTGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_global_planner {

RRTGlobalPlanner::RRTGlobalPlanner() : 
    costmap_ros_(nullptr), costmap_(nullptr), initialized_(false), planning_attempts_(0) {
}

RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(nullptr), costmap_(nullptr), initialized_(false), planning_attempts_(0) {
    initialize(name, costmap_ros);
}

RRTGlobalPlanner::~RRTGlobalPlanner() {
}

void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle nh;
        
        // Get parameters - adjusted for better performance
        private_nh.param("step_size", step_size_, 1.0);
        private_nh.param("goal_tolerance", goal_tolerance_, 1.0);
        private_nh.param("max_iterations", max_iterations_, 10000);
        private_nh.param("collision_check_resolution", collision_check_resolution_, 0.1);
        private_nh.param("use_deterministic", use_deterministic_, true);  // New parameter
        
        // Publishers for visualization
        plan_pub_ = nh.advertise<nav_msgs::Path>("rrt_global_plan", 1);
        tree_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_tree", 1);
        path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_path", 1);
        
        // Initialize with a default seed (will be overridden in makePlan if deterministic)
        std::random_device rd;
        gen_ = std::mt19937(rd());
        
        ROS_INFO("RRT Global Planner initialized successfully");
        ROS_INFO("Parameters: step_size=%.2f, goal_tolerance=%.2f, max_iterations=%d, deterministic=%s", 
                 step_size_, goal_tolerance_, max_iterations_, use_deterministic_ ? "true" : "false");
        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
}

size_t RRTGlobalPlanner::generateSeed(const geometry_msgs::PoseStamped& start, 
                                      const geometry_msgs::PoseStamped& goal,
                                      int attempt_number) {
    // Generate a seed based on start, goal, and attempt number
    size_t seed = 0;
    
    // Hash start position
    seed ^= std::hash<double>{}(start.pose.position.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<double>{}(start.pose.position.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    
    // Hash goal position
    seed ^= std::hash<double>{}(goal.pose.position.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<double>{}(goal.pose.position.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    
    // Include attempt number to get different paths on retries
    seed ^= std::hash<int>{}(attempt_number) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    
    return seed;
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("The planner has not been initialized");
        return false;
    }
    
    plan.clear();
    
    // Check if this is a new goal
    bool is_new_goal = (std::abs(goal.pose.position.x - last_goal_.pose.position.x) > 0.1 ||
                       std::abs(goal.pose.position.y - last_goal_.pose.position.y) > 0.1);
    
    if (is_new_goal) {
        planning_attempts_ = 0;  // Reset attempts for new goal
        last_goal_ = goal;
    } else {
        planning_attempts_++;    // Increment attempts for same goal
    }
    
    // Setup random number generator
    if (use_deterministic_) {
        size_t seed = generateSeed(start, goal, planning_attempts_);
        gen_ = std::mt19937(seed);
        ROS_INFO("Using deterministic seed based on start/goal/attempt: %zu (attempt %d)", 
                 seed, planning_attempts_);
    } else {
        std::random_device rd;
        gen_ = std::mt19937(rd());
    }
    
    // Update distributions with current costmap bounds
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    double width = costmap_->getSizeInMetersX();
    double height = costmap_->getSizeInMetersY();
    
    dis_x_ = std::uniform_real_distribution<>(origin_x, origin_x + width);
    dis_y_ = std::uniform_real_distribution<>(origin_y, origin_y + height);
    
    // Check if start and goal are valid
    if (!isPointValid(start.pose.position.x, start.pose.position.y)) {
        ROS_ERROR("Start position is invalid");
        return false;
    }
    
    if (!isPointValid(goal.pose.position.x, goal.pose.position.y)) {
        ROS_ERROR("Goal position is invalid");
        return false;
    }
    
    // Calculate distance for adaptive parameters
    double total_distance = distance(start.pose.position.x, start.pose.position.y,
                                   goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("Planning path from (%.2f, %.2f) to (%.2f, %.2f), distance: %.2f",
             start.pose.position.x, start.pose.position.y,
             goal.pose.position.x, goal.pose.position.y, total_distance);
    
    // Adaptive step size based on distance
    double adaptive_step = std::min(step_size_, total_distance / 10.0);
    adaptive_step = std::max(adaptive_step, 0.2);  // Minimum step size
    
    // Initialize RRT tree with start node
    std::vector<Node> tree;
    tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);
    
    bool goal_reached = false;
    int goal_node_id = -1;
    
    // Adaptive goal bias - increase with attempts for difficult goals
    double base_goal_bias = std::min(0.3, 0.1 + (total_distance / 50.0));
    double goal_bias = std::min(0.5, base_goal_bias + (planning_attempts_ * 0.05));
    
    // RRT main loop
    for (int i = 0; i < max_iterations_; ++i) {
        // Sample random point (with adaptive goal bias)
        double rand_x, rand_y;
        double rand_val = dis_x_(gen_);
        
        if (rand_val < goal_bias) { // Goal bias
            rand_x = goal.pose.position.x;
            rand_y = goal.pose.position.y;
        } else {
            // Biased sampling towards goal direction
            if (rand_val < goal_bias * 2) { // Additional directional bias
                double angle = std::atan2(goal.pose.position.y - start.pose.position.y,
                                        goal.pose.position.x - start.pose.position.x);
                double dist = dis_x_(gen_) * total_distance;
                rand_x = start.pose.position.x + dist * std::cos(angle);
                rand_y = start.pose.position.y + dist * std::sin(angle);
            } else {
                rand_x = dis_x_(gen_);
                rand_y = dis_y_(gen_);
            }
        }
        
        // Find nearest node in tree
        int nearest_id = getNearestNode(tree, rand_x, rand_y);
        
        // Steer towards sampled point with adaptive step size
        Node new_node = steer(tree[nearest_id], rand_x, rand_y);
        new_node.parent_id = nearest_id;
        
        // Check collision with relaxed checking for long distances
        if (isCollisionFree(tree[nearest_id].x, tree[nearest_id].y, new_node.x, new_node.y)) {
            tree.push_back(new_node);
            
            // Check if we reached the goal
            double dist_to_goal = distance(new_node.x, new_node.y, 
                                         goal.pose.position.x, goal.pose.position.y);
            if (dist_to_goal < goal_tolerance_) {
                goal_reached = true;
                goal_node_id = tree.size() - 1;
                ROS_INFO("Goal reached after %d iterations", i);
                break;
            }
            
            // Try to connect directly to goal if close enough
            if (dist_to_goal < step_size_ * 3) {
                if (isCollisionFree(new_node.x, new_node.y, 
                                  goal.pose.position.x, goal.pose.position.y)) {
                    Node goal_node(goal.pose.position.x, goal.pose.position.y, tree.size() - 1);
                    tree.push_back(goal_node);
                    goal_reached = true;
                    goal_node_id = tree.size() - 1;
                    ROS_INFO("Connected directly to goal after %d iterations", i);
                    break;
                }
            }
        }
        
        // Publish tree periodically for visualization
        if (i % 100 == 0) {
            publishTree(tree);
            ROS_INFO_THROTTLE(1.0, "RRT iteration %d/%d, tree size: %zu, goal_bias: %.2f", 
                             i, max_iterations_, tree.size(), goal_bias);
        }
    }
    
    if (!goal_reached) {
        ROS_WARN("Failed to reach goal after %d iterations (attempt %d)", 
                 max_iterations_, planning_attempts_);
        // Find closest node to goal
        goal_node_id = getNearestNode(tree, goal.pose.position.x, goal.pose.position.y);
        double final_dist = distance(tree[goal_node_id].x, tree[goal_node_id].y,
                                   goal.pose.position.x, goal.pose.position.y);
        ROS_WARN("Closest node to goal is %.2f meters away", final_dist);
        
        // If we're reasonably close, use the partial path
        if (final_dist > total_distance * 0.8) {
            ROS_ERROR("Could not find a valid path to goal");
            return false;
        }
    }
    
    // Extract path from tree
    plan = extractPath(tree, goal_node_id, start, goal);
    
    if (plan.empty()) {
        ROS_ERROR("Failed to extract path from RRT tree");
        return false;
    }
    
    // Smooth the path
    plan = smoothPath(plan);
    
    ROS_INFO("Path found with %zu waypoints", plan.size());
    
    // Publish final tree and path
    publishTree(tree);
    publishPath(plan);
    
    // Publish the plan
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = plan;
    plan_pub_.publish(gui_path);
    
    return true;
}

double RRTGlobalPlanner::distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

int RRTGlobalPlanner::getNearestNode(const std::vector<Node>& tree, double x, double y) {
    int nearest_id = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < tree.size(); ++i) {
        double dist = distance(tree[i].x, tree[i].y, x, y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = i;
        }
    }
    
    return nearest_id;
}

Node RRTGlobalPlanner::steer(const Node& from, double to_x, double to_y) {
    double dist = distance(from.x, from.y, to_x, to_y);
    
    if (dist <= step_size_) {
        return Node(to_x, to_y, -1);
    }
    
    double ratio = step_size_ / dist;
    double new_x = from.x + ratio * (to_x - from.x);
    double new_y = from.y + ratio * (to_y - from.y);
    
    return Node(new_x, new_y, -1);
}

bool RRTGlobalPlanner::isCollisionFree(double x1, double y1, double x2, double y2) {
    // First check if endpoints are valid
    if (!isPointValid(x1, y1) || !isPointValid(x2, y2)) {
        return false;
    }
    
    double dist = distance(x1, y1, x2, y2);
    
    // For very short segments, just check endpoints
    if (dist < collision_check_resolution_ * 2) {
        return true;
    }
    
    // Adaptive collision checking - use fewer checks for longer segments
    double check_resolution = collision_check_resolution_;
    if (dist > step_size_ * 2) {
        check_resolution = std::min(dist / 10.0, collision_check_resolution_ * 2);
    }
    
    int num_checks = static_cast<int>(dist / check_resolution);
    num_checks = std::max(2, num_checks);  // At least check midpoint
    
    for (int i = 1; i < num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        double check_x = x1 + t * (x2 - x1);
        double check_y = y1 + t * (y2 - y1);
        
        if (!isPointValid(check_x, check_y)) {
            return false;
        }
    }
    
    return true;
}

bool RRTGlobalPlanner::isPointValid(double x, double y) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
        return false;
    }
    
    unsigned char cost = costmap_->getCost(mx, my);
    
    // Be more permissive for path planning
    return cost < costmap_2d::LETHAL_OBSTACLE;
}

std::vector<geometry_msgs::PoseStamped> RRTGlobalPlanner::extractPath(
    const std::vector<Node>& tree, int goal_node_id,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal) {
    
    std::vector<geometry_msgs::PoseStamped> path;
    
    if (goal_node_id < 0 || goal_node_id >= tree.size()) {
        ROS_ERROR("Invalid goal node id: %d", goal_node_id);
        return path;
    }
    
    // Trace back from goal to start
    std::vector<int> path_indices;
    int current_id = goal_node_id;
    
    while (current_id >= 0) {
        path_indices.push_back(current_id);
        current_id = tree[current_id].parent_id;
    }
    
    // Reverse to get path from start to goal
    std::reverse(path_indices.begin(), path_indices.end());
    
    // Create pose stamped path
    for (size_t i = 0; i < path_indices.size(); ++i) {
        int idx = path_indices[i];
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = tree[idx].x;
        pose.pose.position.y = tree[idx].y;
        pose.pose.position.z = 0.0;
        
        // Calculate orientation
        if (i < path_indices.size() - 1) {
            int next_idx = path_indices[i + 1];
            double yaw = std::atan2(tree[next_idx].y - tree[idx].y, 
                                   tree[next_idx].x - tree[idx].x);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        } else {
            pose.pose.orientation = goal.pose.orientation;
        }
        
        path.push_back(pose);
    }
    
    // Add the actual goal if we didn't reach it exactly
    if (path.size() > 0) {
        double last_dist = distance(path.back().pose.position.x, path.back().pose.position.y,
                                  goal.pose.position.x, goal.pose.position.y);
        if (last_dist > 0.1) {
            path.push_back(goal);
        }
    }
    
    return path;
}

std::vector<geometry_msgs::PoseStamped> RRTGlobalPlanner::smoothPath(
    const std::vector<geometry_msgs::PoseStamped>& path) {
    
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<geometry_msgs::PoseStamped> smooth_path;
    smooth_path.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        bool found_shortcut = false;
        
        // Try to find the farthest point we can reach directly
        while (j > i + 1) {
            if (isCollisionFree(path[i].pose.position.x, path[i].pose.position.y,
                               path[j].pose.position.x, path[j].pose.position.y)) {
                i = j;
                found_shortcut = true;
                break;
            }
            j--;
        }
        
        if (!found_shortcut) {
            i++;
        }
        
        if (i < path.size()) {
            smooth_path.push_back(path[i]);
        }
    }
    
    return smooth_path;
}

// // Key improvements for collision avoidance:
// // 1. Better collision checking with robot footprint consideration
// // 2. Improved obstacle cost evaluation
// // 3. Enhanced path smoothing with safety margins
// // 4. Corner-aware collision detection

// bool RRTGlobalPlanner::isPointValid(double x, double y) {
//     unsigned int mx, my;
//     if (!costmap_->worldToMap(x, y, mx, my)) {
//         return false;
//     }
    
//     // Get robot footprint radius (approximate as circle for simplicity)
//     double robot_radius = 0.35; // Adjust based on your robot size
    
//     // Check multiple points around the robot's footprint
//     int check_radius = static_cast<int>(robot_radius / costmap_->getResolution()) + 1;
    
//     for (int dx = -check_radius; dx <= check_radius; dx++) {
//         for (int dy = -check_radius; dy <= check_radius; dy++) {
//             // Only check points within the robot's circular footprint
//             if (dx*dx + dy*dy <= check_radius*check_radius) {
//                 unsigned int check_mx = mx + dx;
//                 unsigned int check_my = my + dy;
                
//                 // Skip if outside costmap bounds
//                 if (check_mx >= costmap_->getSizeInCellsX() || 
//                     check_my >= costmap_->getSizeInCellsY()) {
//                     continue;
//                 }
                
//                 unsigned char cost = costmap_->getCost(check_mx, check_my);
                
//                 // More conservative obstacle avoidance
//                 if (cost >= costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
//                     return false;
//                 }
                
//                 // Add penalty for high cost areas (near obstacles)
//                 if (cost > 127) { // High cost threshold
//                     return false;
//                 }
//             }
//         }
//     }
    
//     return true;
// }

// bool RRTGlobalPlanner::isCollisionFree(double x1, double y1, double x2, double y2) {
//     // First check if endpoints are valid with footprint consideration
//     if (!isPointValid(x1, y1) || !isPointValid(x2, y2)) {
//         return false;
//     }
    
//     double dist = distance(x1, y1, x2, y2);
    
//     // For very short segments, endpoint check is sufficient
//     if (dist < 0.1) {
//         return true;
//     }
    
//     // Use finer collision checking resolution
//     double check_resolution = std::min(collision_check_resolution_, 0.1);
//     int num_checks = static_cast<int>(dist / check_resolution);
//     num_checks = std::max(3, num_checks); // Minimum 3 intermediate checks
    
//     // Check intermediate points along the path
//     for (int i = 1; i < num_checks; ++i) {
//         double t = static_cast<double>(i) / num_checks;
//         double check_x = x1 + t * (x2 - x1);
//         double check_y = y1 + t * (y2 - y1);
        
//         if (!isPointValid(check_x, check_y)) {
//             return false;
//         }
//     }
    
//     // Additional corner collision check
//     return checkCornerCollision(x1, y1, x2, y2);
// }

// bool RRTGlobalPlanner::checkCornerCollision(double x1, double y1, double x2, double y2) {
//     double robot_radius = 0.35; // Robot radius
    
//     // Get all cells the line passes through using Bresenham-like algorithm
//     unsigned int mx1, my1, mx2, my2;
//     if (!costmap_->worldToMap(x1, y1, mx1, my1) || 
//         !costmap_->worldToMap(x2, y2, mx2, my2)) {
//         return false;
//     }
    
//     // Check cells along the line
//     int dx = abs(static_cast<int>(mx2) - static_cast<int>(mx1));
//     int dy = abs(static_cast<int>(my2) - static_cast<int>(my1));
//     int x = mx1;
//     int y = my1;
//     int n = 1 + dx + dy;
//     int x_inc = (mx1 < mx2) ? 1 : -1;
//     int y_inc = (my1 < my2) ? 1 : -1;
//     int error = dx - dy;
    
//     dx *= 2;
//     dy *= 2;
    
//     for (; n > 0; --n) {
//         // Check current cell and surrounding area for robot footprint
//         double wx, wy;
//         costmap_->mapToWorld(x, y, wx, wy);
        
//         if (!isPointValid(wx, wy)) {
//             return false;
//         }
        
//         if (error > 0) {
//             x += x_inc;
//             error -= dy;
//         } else {
//             y += y_inc;
//             error += dx;
//         }
//     }
    
//     return true;
// }

// std::vector<geometry_msgs::PoseStamped> RRTGlobalPlanner::smoothPath(
//     const std::vector<geometry_msgs::PoseStamped>& path) {
    
//     if (path.size() <= 2) {
//         return path;
//     }
    
//     std::vector<geometry_msgs::PoseStamped> smooth_path;
//     smooth_path.push_back(path[0]);
    
//     size_t i = 0;
//     while (i < path.size() - 1) {
//         size_t j = path.size() - 1;
//         bool found_shortcut = false;
        
//         // Try to find the farthest point we can reach directly
//         while (j > i + 1) {
//             // Use more conservative collision checking for path smoothing
//             if (isCollisionFreeWithMargin(path[i].pose.position.x, path[i].pose.position.y,
//                                          path[j].pose.position.x, path[j].pose.position.y, 0.2)) {
//                 i = j;
//                 found_shortcut = true;
//                 break;
//             }
//             j--;
//         }
        
//         if (!found_shortcut) {
//             i++;
//         }
        
//         if (i < path.size()) {
//             smooth_path.push_back(path[i]);
//         }
//     }
    
//     // Add intermediate waypoints for sharp turns to help local planner
//     return addIntermediateWaypoints(smooth_path);
// }

// bool RRTGlobalPlanner::isCollisionFreeWithMargin(double x1, double y1, double x2, double y2, double margin) {
//     // Expand the collision checking with additional safety margin
//     double expanded_radius = 0.35 + margin; // Robot radius + safety margin
    
//     double dist = distance(x1, y1, x2, y2);
//     if (dist < 0.05) return true;
    
//     int num_checks = static_cast<int>(dist / 0.05); // Very fine resolution
//     num_checks = std::max(5, num_checks);
    
//     for (int i = 0; i <= num_checks; ++i) {
//         double t = static_cast<double>(i) / num_checks;
//         double check_x = x1 + t * (x2 - x1);
//         double check_y = y1 + t * (y2 - y1);
        
//         if (!isPointValidWithRadius(check_x, check_y, expanded_radius)) {
//             return false;
//         }
//     }
    
//     return true;
// }

// bool RRTGlobalPlanner::isPointValidWithRadius(double x, double y, double radius) {
//     unsigned int mx, my;
//     if (!costmap_->worldToMap(x, y, mx, my)) {
//         return false;
//     }
    
//     int check_radius = static_cast<int>(radius / costmap_->getResolution()) + 1;
    
//     for (int dx = -check_radius; dx <= check_radius; dx++) {
//         for (int dy = -check_radius; dy <= check_radius; dy++) {
//             if (dx*dx + dy*dy <= check_radius*check_radius) {
//                 unsigned int check_mx = mx + dx;
//                 unsigned int check_my = my + dy;
                
//                 if (check_mx >= costmap_->getSizeInCellsX() || 
//                     check_my >= costmap_->getSizeInCellsY()) {
//                     return false; // Conservative approach for boundary
//                 }
                
//                 unsigned char cost = costmap_->getCost(check_mx, check_my);
                
//                 if (cost >= costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
//                     return false;
//                 }
                
//                 // More conservative for smoothed paths
//                 if (cost > 100) {
//                     return false;
//                 }
//             }
//         }
//     }
    
//     return true;
// }

// std::vector<geometry_msgs::PoseStamped> RRTGlobalPlanner::addIntermediateWaypoints(
//     const std::vector<geometry_msgs::PoseStamped>& path) {
    
//     if (path.size() <= 2) return path;
    
//     std::vector<geometry_msgs::PoseStamped> enhanced_path;
//     enhanced_path.push_back(path[0]);
    
//     for (size_t i = 1; i < path.size(); ++i) {
//         // Check angle change between segments
//         if (i < path.size() - 1) {
//             double angle1 = std::atan2(path[i].pose.position.y - path[i-1].pose.position.y,
//                                      path[i].pose.position.x - path[i-1].pose.position.x);
//             double angle2 = std::atan2(path[i+1].pose.position.y - path[i].pose.position.y,
//                                      path[i+1].pose.position.x - path[i].pose.position.x);
            
//             double angle_diff = std::abs(angles::shortest_angular_distance(angle1, angle2));
            
//             // Add intermediate points for sharp turns
//             if (angle_diff > M_PI/4) { // 45 degree threshold
//                 double dist = distance(path[i-1].pose.position.x, path[i-1].pose.position.y,
//                                      path[i].pose.position.x, path[i].pose.position.y);
                
//                 if (dist > 1.0) { // Only for longer segments
//                     geometry_msgs::PoseStamped intermediate = path[i-1];
//                     intermediate.pose.position.x += 0.7 * (path[i].pose.position.x - path[i-1].pose.position.x);
//                     intermediate.pose.position.y += 0.7 * (path[i].pose.position.y - path[i-1].pose.position.y);
                    
//                     // Update orientation
//                     double yaw = std::atan2(path[i].pose.position.y - intermediate.pose.position.y,
//                                           path[i].pose.position.x - intermediate.pose.position.x);
//                     tf2::Quaternion q;
//                     q.setRPY(0, 0, yaw);
//                     intermediate.pose.orientation.x = q.x();
//                     intermediate.pose.orientation.y = q.y();
//                     intermediate.pose.orientation.z = q.z();
//                     intermediate.pose.orientation.w = q.w();
                    
//                     enhanced_path.push_back(intermediate);
//                 }
//             }
//         }
        
//         enhanced_path.push_back(path[i]);
//     }
    
//     return enhanced_path;
// }

void RRTGlobalPlanner::publishTree(const std::vector<Node>& tree) {
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.ns = "rrt_tree_edges";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Tree edges
    visualization_msgs::Marker edges;
    edges.header.frame_id = costmap_ros_->getGlobalFrameID();
    edges.header.stamp = ros::Time::now();
    edges.ns = "rrt_tree_edges";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.scale.x = 0.02;
    edges.color.r = 0.0;
    edges.color.g = 0.0;
    edges.color.b = 1.0;
    edges.color.a = 0.5;
    edges.pose.orientation.w = 1.0;
    
    for (size_t i = 1; i < tree.size(); ++i) {
        if (tree[i].parent_id >= 0) {
            geometry_msgs::Point p1, p2;
            p1.x = tree[tree[i].parent_id].x;
            p1.y = tree[tree[i].parent_id].y;
            p1.z = 0.0;
            p2.x = tree[i].x;
            p2.y = tree[i].y;
            p2.z = 0.0;
            edges.points.push_back(p1);
            edges.points.push_back(p2);
        }
    }
    
    if (!edges.points.empty()) {
        marker_array.markers.push_back(edges);
    }
    
    // Tree nodes (only show a subset for performance)
    if (tree.size() < 1000) {
        visualization_msgs::Marker nodes;
        nodes.header.frame_id = costmap_ros_->getGlobalFrameID();
        nodes.header.stamp = ros::Time::now();
        nodes.ns = "rrt_tree_nodes";
        nodes.id = 1;
        nodes.type = visualization_msgs::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::Marker::ADD;
        nodes.scale.x = 0.05;
        nodes.scale.y = 0.05;
        nodes.scale.z = 0.05;
        nodes.color.r = 0.0;
        nodes.color.g = 1.0;
        nodes.color.b = 0.0;
        nodes.color.a = 1.0;
        nodes.pose.orientation.w = 1.0;
        
        for (const auto& node : tree) {
            geometry_msgs::Point p;
            p.x = node.x;
            p.y = node.y;
            p.z = 0.0;
            nodes.points.push_back(p);
        }
        
        marker_array.markers.push_back(nodes);
    }
    
    tree_pub_.publish(marker_array);
}

void RRTGlobalPlanner::publishPath(const std::vector<geometry_msgs::PoseStamped>& path) {
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "rrt_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.1;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    path_marker.pose.orientation.w = 1.0;
    
    for (const auto& pose : path) {
        geometry_msgs::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.0;
        path_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(path_marker);
    path_pub_.publish(marker_array);
}

} // namespace rrt_global_planner

