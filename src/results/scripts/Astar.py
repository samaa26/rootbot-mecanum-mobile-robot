#!/usr/bin/env python3
import rospy
import actionlib
import time
import psutil
import os
import threading
import csv

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

cpu_usages = []
memory_usages = []
goal_reached = [False]

CSV_FILENAME = "/home/samaa/building_B_results.csv"

def get_plan(start, goal, tolerance=0.0):
    rospy.wait_for_service('/move_base/make_plan')
    try:
        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        return get_plan(start=start, goal=goal, tolerance=tolerance).plan
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def calculate_path_distance(path):
    distance = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        x0, y0 = poses[i-1].pose.position.x, poses[i-1].pose.position.y
        x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
        distance += ((x1 - x0)**2 + (y1 - y0)**2)**0.5
    return distance

def monitor_resources():
    while not rospy.is_shutdown():
        cpu_usages.append(psutil.cpu_percent(interval=0.5))
        mem_usage = get_move_base_memory_usage()
        if mem_usage is not None:
            memory_usages.append(mem_usage)
        if goal_reached[0]:
            break

def get_move_base_memory_usage():
    for proc in psutil.process_iter(['name', 'memory_info']):
        if proc.info['name'] == 'move_base':
            mem_bytes = proc.info['memory_info'].rss
            mem_mb = mem_bytes / (1024 * 1024)  # convert bytes to MB
            return mem_mb
    return None

def save_metrics_to_csv(planning_time, execution_time, path_distance, avg_cpu, avg_mem):
    file_exists = os.path.isfile(CSV_FILENAME)

    with open(CSV_FILENAME, mode='a', newline='') as csvfile:
        fieldnames = ["Run #", "Planning Time (s)", "Execution Time (s)", "Total Distance (m)", "Avg CPU Usage (%)", "Avg Memory Usage (MB)"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        # Determine run number by counting lines (excluding header)
        run_number = sum(1 for line in open(CSV_FILENAME)) if file_exists else 0
        run_number += 1

        writer.writerow({
            "Run #": run_number,
            "Planning Time (s)": planning_time,
            "Execution Time (s)": execution_time,
            "Total Distance (m)": path_distance,
            "Avg CPU Usage (%)": avg_cpu,
            "Avg Memory Usage (MB)": avg_mem
        })

    rospy.loginfo(f"Metrics appended to {CSV_FILENAME}")

def send_goal_and_log():
    global goal_reached
    rospy.init_node('send_goal_and_measure', anonymous=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 6.18           #9.9               #6.18                    #11.3                   
    goal.target_pose.pose.position.y = -3.56           #2.55             #-3.56                    #-1.4                 
    goal.target_pose.pose.orientation.w = 1.0

    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.header.stamp = rospy.Time.now()
    start_pose.pose.position.x = 0.0
    start_pose.pose.position.y = 0.0
    start_pose.pose.orientation.w = 1.0

    threading.Thread(target=monitor_resources).start()

    rospy.loginfo("Requesting path plan...")
    t0 = time.time()
    path = get_plan(start_pose, goal.target_pose)
    t1 = time.time()
    planning_time = t1 - t0
    rospy.loginfo(f"Path planning completed in {planning_time:.2f} seconds")

    path_distance = calculate_path_distance(path)
    rospy.loginfo(f"Total path distance: {path_distance:.2f} meters")

    rospy.loginfo("Sending goal to move_base...")
    t_start = time.time()
    client.send_goal(goal)
    client.wait_for_result()
    t_end = time.time()
    goal_reached[0] = True

    execution_time = t_end - t_start
    rospy.loginfo(f"Goal reached in {execution_time:.2f} seconds")

    avg_cpu = sum(cpu_usages) / len(cpu_usages) if cpu_usages else 0.0
    avg_mem = sum(memory_usages) / len(memory_usages) if memory_usages else 0.0
    rospy.loginfo(f"Average CPU usage: {avg_cpu:.2f}%")
    rospy.loginfo(f"Average Memory usage: {avg_mem:.2f} MB")

    save_metrics_to_csv(planning_time, execution_time, path_distance, avg_cpu, avg_mem)

if __name__ == '__main__':
    try:
        send_goal_and_log()
    except rospy.ROSInterruptException:
        pass