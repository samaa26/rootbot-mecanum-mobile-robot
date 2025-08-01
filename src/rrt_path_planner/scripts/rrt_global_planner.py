#!/usr/bin/env python3

import rospy
import numpy as np
import math
import random
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_core import BaseGlobalPlanner
import tf2_ros
from nav_msgs.srv import GetPlan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Node:
    """RRT Node class"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTGlobalPlanner(BaseGlobalPlanner):
    """RRT Global Planner Plugin for move_base"""
    
    def __init__(self):
        """Constructor"""
        self.costmap = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        
        # RRT parameters
        self.max_iterations = 5000
        self.step_size = 0.5
        self.goal_tolerance = 0.5
        self.obstacle_clearance = 0.3
        
        # Publishers for visualization
        self.path_pub = None
        self.tree_pub = None
        self.nodes_pub = None
        
        # Tree storage
        self.nodes = []
        
    def initialize(self, name, costmap_ros):
        """Initialize the planner"""
        self.costmap_ros = costmap_ros
        self.costmap = self.costmap_ros.getCostmap()
        self.global_frame = costmap_ros.getGlobalFrameID()
        
        # Get costmap properties
        self.origin_x = self.costmap.getOriginX()
        self.origin_y = self.costmap.getOriginY()
        self.resolution = self.costmap.getResolution()
        self.width = self.costmap.getSizeInCellsX()
        self.height = self.costmap.getSizeInCellsY()
        
        # Initialize publishers
        self.path_pub = rospy.Publisher('/rrt_path', Path, queue_size=1)
        self.tree_pub = rospy.Publisher('/rrt_tree', MarkerArray, queue_size=1)
        self.nodes_pub = rospy.Publisher('/rrt_nodes', MarkerArray, queue_size=1)
        
        rospy.loginfo("RRT Global Planner initialized successfully")
        return True
        
    def makePlan(self, start, goal, tolerance=0.0):
        """Create a plan from start to goal"""
        rospy.loginfo("RRT Global Planner: Making new plan")
        
        # Clear previous tree
        self.nodes = []
        
        # Convert poses to map coordinates
        start_x, start_y = self.world_to_map(start.pose.position.x, start.pose.position.y)
        goal_x, goal_y = self.world_to_map(goal.pose.position.x, goal.pose.position.y)
        
        # Check if start and goal are valid
        if not self.is_valid_point(start_x, start_y) or not self.is_valid_point(goal_x, goal_y):
            rospy.logwarn("Start or goal position is invalid")
            return []
        
        # Initialize RRT with start node
        start_node = Node(start_x, start_y)
        self.nodes.append(start_node)
        
        # RRT main loop
        for i in range(self.max_iterations):
            # Sample random point
            if random.random() < 0.1:  # 10% bias towards goal
                rand_x, rand_y = goal_x, goal_y
            else:
                rand_x = random.randint(0, self.width - 1)
                rand_y = random.randint(0, self.height - 1)
            
            # Find nearest node
            nearest_node = self.get_nearest_node(rand_x, rand_y)
            
            # Steer towards random point
            new_node = self.steer(nearest_node, rand_x, rand_y)
            
            # Check if new node is valid
            if self.is_collision_free(nearest_node, new_node):
                self.nodes.append(new_node)
                
                # Visualize tree growth
                if i % 10 == 0:
                    self.visualize_tree()
                    self.visualize_nodes()
                
                # Check if we reached the goal
                dist_to_goal = self.distance(new_node.x, new_node.y, goal_x, goal_y)
                if dist_to_goal < self.goal_tolerance / self.resolution:
                    rospy.loginfo(f"Goal reached after {i} iterations")
                    
                    # Create final node at exact goal position
                    goal_node = Node(goal_x, goal_y)
                    goal_node.parent = new_node
                    self.nodes.append(goal_node)
                    
                    # Extract and return path
                    path = self.extract_path(goal_node, start, goal)
                    self.visualize_path(path)
                    return path
        
        rospy.logwarn("RRT failed to find a path within iteration limit")
        return []
    
    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map coordinates"""
        map_x = int((world_x - self.origin_x) / self.resolution)
        map_y = int((world_y - self.origin_y) / self.resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convert map coordinates to world coordinates"""
        world_x = map_x * self.resolution + self.origin_x
        world_y = map_y * self.resolution + self.origin_y
        return world_x, world_y
    
    def is_valid_point(self, x, y):
        """Check if a point is valid (within map and not in obstacle)"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        
        # Check obstacle clearance
        for dx in range(-int(self.obstacle_clearance/self.resolution), 
                       int(self.obstacle_clearance/self.resolution)+1):
            for dy in range(-int(self.obstacle_clearance/self.resolution), 
                           int(self.obstacle_clearance/self.resolution)+1):
                check_x = x + dx
                check_y = y + dy
                if (check_x >= 0 and check_x < self.width and 
                    check_y >= 0 and check_y < self.height):
                    cost = self.costmap.getCost(check_x, check_y)
                    if cost >= 100:  # Obstacle threshold
                        return False
        return True
    
    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def get_nearest_node(self, x, y):
        """Find nearest node in the tree"""
        min_dist = float('inf')
        nearest_node = None
        
        for node in self.nodes:
            dist = self.distance(node.x, node.y, x, y)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node
    
    def steer(self, from_node, to_x, to_y):
        """Steer from node towards target position"""
        dist = self.distance(from_node.x, from_node.y, to_x, to_y)
        
        if dist < self.step_size / self.resolution:
            new_x = to_x
            new_y = to_y
        else:
            ratio = (self.step_size / self.resolution) / dist
            new_x = from_node.x + ratio * (to_x - from_node.x)
            new_y = from_node.y + ratio * (to_y - from_node.y)
        
        new_node = Node(int(new_x), int(new_y))
        new_node.parent = from_node
        new_node.cost = from_node.cost + self.distance(from_node.x, from_node.y, new_x, new_y)
        
        return new_node
    
    def is_collision_free(self, from_node, to_node):
        """Check if path between two nodes is collision-free"""
        # Use Bresenham's line algorithm
        x0, y0 = from_node.x, from_node.y
        x1, y1 = to_node.x, to_node.y
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            if not self.is_valid_point(x0, y0):
                return False
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return True
    
    def extract_path(self, goal_node, start_pose, goal_pose):
        """Extract path from goal node to start"""
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = rospy.Time.now()
        
        # Trace back from goal to start
        current = goal_node
        path_nodes = []
        
        while current is not None:
            path_nodes.append(current)
            current = current.parent
        
        # Reverse to get path from start to goal
        path_nodes.reverse()
        
        # Convert to PoseStamped messages
        for node in path_nodes:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = rospy.Time.now()
            
            # Convert back to world coordinates
            world_x, world_y = self.map_to_world(node.x, node.y)
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            
            # Set orientation (for simplicity, use goal orientation)
            pose.pose.orientation = goal_pose.pose.orientation
            
            path.poses.append(pose)
        
        return path.poses
    
    def visualize_tree(self):
        """Visualize RRT tree in RViz"""
        marker_array = MarkerArray()
        
        for i, node in enumerate(self.nodes):
            if node.parent is not None:
                marker = Marker()
                marker.header.frame_id = self.global_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "rrt_tree"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.01
                marker.color.a = 0.5
                marker.color.g = 1.0
                
                # Add line from parent to current node
                p1 = Point()
                p1.x, p1.y = self.map_to_world(node.parent.x, node.parent.y)
                p1.z = 0.1
                
                p2 = Point()
                p2.x, p2.y = self.map_to_world(node.x, node.y)
                p2.z = 0.1
                
                marker.points.append(p1)
                marker.points.append(p2)
                
                marker_array.markers.append(marker)
        
        self.tree_pub.publish(marker_array)
    
    def visualize_nodes(self):
        """Visualize RRT nodes in RViz"""
        marker_array = MarkerArray()
        
        for i, node in enumerate(self.nodes):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rrt_nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 0.8
            marker.color.b = 1.0
            
            world_x, world_y = self.map_to_world(node.x, node.y)
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker_array.markers.append(marker)
        
        self.nodes_pub.publish(marker_array)
    
    def visualize_path(self, path):
        """Visualize final path in RViz"""
        if not path:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = path
        
        self.path_pub.publish(path_msg)


class RRTNavigator:
    """Class to handle navigation using RRT planner"""
    
    def __init__(self):
        rospy.init_node('rrt_navigator', anonymous=True)
        
        # Create action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Subscribe to goal topic
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("RRT Navigator initialized. Click '2D Nav Goal' in RViz to set a goal.")
    
    def goal_callback(self, msg):
        """Callback for receiving navigation goals"""
        rospy.loginfo("Received new navigation goal")
        
        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = msg.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = msg.pose
        
        # Send goal to move_base
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Goal sent to move_base with RRT planner")
        
        # Wait for result
        wait = self.move_base_client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            self.move_base_client.cancel_goal()
        else:
            result = self.move_base_client.get_result()
            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
            else:
                rospy.logwarn("Failed to reach goal")


if __name__ == '__main__':
    try:
        navigator = RRTNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass