#!/usr/bin/env python3

import rospy
import numpy as np
import random
import math
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
import tf2_ros
import tf2_geometry_msgs

class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTPathPlanner:
    def __init__(self):
        rospy.init_node('rrt_path_planner', anonymous=True)
        
        # Parameters
        self.max_iterations = 5000
        self.step_size = 0.5  # meters
        self.goal_tolerance = 0.3  # meters
        self.obstacle_threshold = 50  # occupancy grid threshold (0-100)
        
        # Map parameters
        self.map_data = None
        self.map_info = None
        self.map_frame = "map"
        
        # RRT tree
        self.nodes = []
        self.start_node = None
        self.goal_node = None
        
        # Publishers
        self.marker_pub = rospy.Publisher('/rrt_visualization', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/rrt_path', MarkerArray, queue_size=10)
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Get robot's current position as start point
        self.get_robot_position()
        
        rospy.loginfo("RRT Path Planner initialized")
        
    def map_callback(self, msg):
        """Callback for occupancy grid map"""
        self.map_data = msg.data
        self.map_info = msg.info
        rospy.loginfo("Map received: %dx%d, resolution: %.3f", 
                     msg.info.width, msg.info.height, msg.info.resolution)
    
    def goal_callback(self, msg):
        """Callback for goal pose from RViz"""
        if self.map_data is None:
            rospy.logwarn("No map received yet!")
            return
            
        # Convert goal to map coordinates
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        rospy.loginfo("Goal received: (%.2f, %.2f)", goal_x, goal_y)
        
        # Get current robot position as start
        start_x, start_y = self.get_robot_position()
        if start_x is None:
            rospy.logwarn("Could not get robot position!")
            return
            
        # Run RRT algorithm
        path = self.plan_path(start_x, start_y, goal_x, goal_y)
        
        if path:
            rospy.loginfo("Path found with %d waypoints", len(path))
            self.publish_path_markers(path)
        else:
            rospy.logwarn("No path found!")
    
    def get_robot_position(self):
        """Get robot's current position from TF"""
        try:
            # Get transform from map to base_link
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, 'base_link', rospy.Time(0), rospy.Duration(1.0))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not get robot position: %s", str(e))
            # Return default start position if TF fails
            return 0.0, 0.0
    
    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map grid coordinates"""
        if self.map_info is None:
            return None, None
            
        map_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convert map grid coordinates to world coordinates"""
        if self.map_info is None:
            return None, None
            
        world_x = map_x * self.map_info.resolution + self.map_info.origin.position.x
        world_y = map_y * self.map_info.resolution + self.map_info.origin.position.y
        
        return world_x, world_y
    
    def is_valid_point(self, x, y):
        """Check if a point is valid (not in obstacle)"""
        if self.map_data is None or self.map_info is None:
            return True
            
        map_x, map_y = self.world_to_map(x, y)
        
        # Check bounds
        if (map_x < 0 or map_x >= self.map_info.width or 
            map_y < 0 or map_y >= self.map_info.height):
            return False
            
        # Check occupancy
        index = map_y * self.map_info.width + map_x
        if index >= len(self.map_data):
            return False
            
        occupancy = self.map_data[index]
        return occupancy < self.obstacle_threshold and occupancy >= 0
    
    def is_path_clear(self, x1, y1, x2, y2):
        """Check if path between two points is clear of obstacles"""
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        steps = int(dist / (self.map_info.resolution * 0.5)) + 1
        
        for i in range(steps + 1):
            t = i / float(steps)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if not self.is_valid_point(x, y):
                return False
                
        return True
    
    def distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def get_nearest_node(self, random_point):
        """Find the nearest node to a random point"""
        min_dist = float('inf')
        nearest_node = None
        
        for node in self.nodes:
            dist = self.distance(node, random_point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node
    
    def steer(self, from_node, to_point):
        """Create new node by steering from one node toward a point"""
        dist = self.distance(from_node, to_point)
        
        if dist <= self.step_size:
            return RRTNode(to_point.x, to_point.y)
        else:
            # Limit step size
            theta = math.atan2(to_point.y - from_node.y, to_point.x - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return RRTNode(new_x, new_y)
    
    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """Main RRT path planning algorithm"""
        # Initialize
        self.nodes = []
        self.start_node = RRTNode(start_x, start_y)
        self.goal_node = RRTNode(goal_x, goal_y)
        self.nodes.append(self.start_node)
        
        rospy.loginfo("Starting RRT from (%.2f, %.2f) to (%.2f, %.2f)", 
                     start_x, start_y, goal_x, goal_y)
        
        # Check if start and goal are valid
        if not self.is_valid_point(start_x, start_y):
            rospy.logerr("Start point is in obstacle!")
            return None
            
        if not self.is_valid_point(goal_x, goal_y):
            rospy.logerr("Goal point is in obstacle!")
            return None
        
        # RRT main loop
        for i in range(self.max_iterations):
            # Generate random point (bias toward goal 10% of time)
            if random.random() < 0.1:
                random_point = self.goal_node
            else:
                # Random point in map bounds
                if self.map_info:
                    min_x = self.map_info.origin.position.x
                    max_x = min_x + self.map_info.width * self.map_info.resolution
                    min_y = self.map_info.origin.position.y
                    max_y = min_y + self.map_info.height * self.map_info.resolution
                    
                    rand_x = random.uniform(min_x, max_x)
                    rand_y = random.uniform(min_y, max_y)
                else:
                    rand_x = random.uniform(-10, 10)
                    rand_y = random.uniform(-10, 10)
                    
                random_point = RRTNode(rand_x, rand_y)
            
            # Find nearest node
            nearest_node = self.get_nearest_node(random_point)
            
            # Steer toward random point
            new_node = self.steer(nearest_node, random_point)
            
            # Check if new node is valid and path is clear
            if (self.is_valid_point(new_node.x, new_node.y) and 
                self.is_path_clear(nearest_node.x, nearest_node.y, new_node.x, new_node.y)):
                
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)
                self.nodes.append(new_node)
                
                # Check if we reached the goal
                if self.distance(new_node, self.goal_node) <= self.goal_tolerance:
                    if self.is_path_clear(new_node.x, new_node.y, self.goal_node.x, self.goal_node.y):
                        self.goal_node.parent = new_node
                        self.goal_node.cost = new_node.cost + self.distance(new_node, self.goal_node)
                        self.nodes.append(self.goal_node)
                        
                        rospy.loginfo("Goal reached in %d iterations!", i + 1)
                        
                        # Publish tree visualization
                        self.publish_tree_markers()
                        
                        # Extract and return path
                        return self.extract_path()
            
            # Publish tree visualization every 100 iterations
            if i % 100 == 0:
                self.publish_tree_markers()
                rospy.loginfo("RRT iteration %d, nodes: %d", i, len(self.nodes))
        
        rospy.logwarn("RRT failed to find path in %d iterations", self.max_iterations)
        self.publish_tree_markers()
        return None
    
    def extract_path(self):
        """Extract path from goal to start by following parent pointers"""
        path = []
        current = self.goal_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
            
        path.reverse()
        return path
    
    def publish_tree_markers(self):
        """Publish RRT tree as visualization markers"""
        marker_array = MarkerArray()
        
        # Tree edges
        edge_marker = Marker()
        edge_marker.header.frame_id = self.map_frame
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.ns = "rrt_tree"
        edge_marker.id = 0
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.02
        edge_marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)  # Blue, semi-transparent
        
        for node in self.nodes:
            if node.parent is not None:
                # Add line from parent to node
                p1 = Point()
                p1.x, p1.y, p1.z = node.parent.x, node.parent.y, 0.0
                p2 = Point()
                p2.x, p2.y, p2.z = node.x, node.y, 0.0
                edge_marker.points.append(p1)
                edge_marker.points.append(p2)
        
        marker_array.markers.append(edge_marker)
        
        # Tree nodes
        node_marker = Marker()
        node_marker.header.frame_id = self.map_frame
        node_marker.header.stamp = rospy.Time.now()
        node_marker.ns = "rrt_nodes"
        node_marker.id = 1
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.05
        node_marker.scale.y = 0.05
        node_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Green
        
        for node in self.nodes:
            p = Point()
            p.x, p.y, p.z = node.x, node.y, 0.0
            node_marker.points.append(p)
        
        marker_array.markers.append(node_marker)
        
        # Start and goal markers
        start_marker = Marker()
        start_marker.header.frame_id = self.map_frame
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = "start_goal"
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_node.x
        start_marker.pose.position.y = self.start_node.y
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        marker_array.markers.append(start_marker)
        
        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = "start_goal"
        goal_marker.id = 3
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_node.x
        goal_marker.pose.position.y = self.goal_node.y
        goal_marker.pose.position.z = 0.0
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        marker_array.markers.append(goal_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_path_markers(self, path):
        """Publish final path as visualization markers"""
        marker_array = MarkerArray()
        
        # Path line
        path_marker = Marker()
        path_marker.header.frame_id = self.map_frame
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "rrt_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1
        path_marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)  # Magenta
        
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = x, y, 0.1
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # Path waypoints
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = self.map_frame
        waypoint_marker.header.stamp = rospy.Time.now()
        waypoint_marker.ns = "path_waypoints"
        waypoint_marker.id = 1
        waypoint_marker.type = Marker.POINTS
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.1
        waypoint_marker.scale.y = 0.1
        waypoint_marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow
        
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = x, y, 0.1
            waypoint_marker.points.append(p)
        
        marker_array.markers.append(waypoint_marker)
        
        self.path_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        planner = RRTPathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass