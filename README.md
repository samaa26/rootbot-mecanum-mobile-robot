
# Path Planning for a Four-Mecanum Wheeled Mobile Robot

An undergraduate thesis project focused on the development, implementation, and evaluation of various global and local path planning algorithms for an autonomous four-wheeled mecanum mobile robot, using ROS (Robot Operating System).

---

## 📌 Overview  
This project presents the design, realization, and performance testing of a mecanum-wheeled mobile robot capable of autonomous navigation in dynamic indoor environments. The robot integrates multiple sensors and leverages various planning algorithms to achieve efficient and robust navigation.

---

## 🔧 Features  
- Full autonomous navigation system  
- Four-mecanum wheel omnidirectional mobility  
- Global Path Planners: Dijkstra, A*, Optimized RRT  
- Local Path Planners: Dynamic Window Approach (DWA), Time Elastic Band (TEB)  
- Sensor Fusion: Wheel encoders, IMU, and LiDAR  
- Localization via Grid-Based SLAM and Extended Kalman Filter (EKF)  
- Low-level motion control using PID controller  
- ROS-based system architecture with RViz visualization  

---

## 🚗 Hardware Setup  
- **Platform**: 4-wheeled mecanum chassis  
- **Actuation**: DC motors with wheel encoders  
- **Sensors**:  
  - IMU (Inertial Measurement Unit)  
  - 2D LiDAR (RPLiDAR A1)  
  - Wheel Encoders  
- **Microcontroller**: Raspberry Pi 4 (8 GB)  
- **Power**: LiPo battery (12V) and power bank for auxiliary components  
- **Software**: ROS Noetic, RViz, Python/C++

### Robot Components  
![Robot Hardware](rootbot-mecanum-mobile-robot/images/Robot.png)  
*Key components: Raspberry Pi, LiDAR, IMU (MPU6050), mecanum wheels (80 mm), motor drivers, and power systems.*  

---

## ⚡ Electrical Setup  
The electrical architecture integrates power distribution, sensor interfacing, and motor control:  
- **Power System**:  
  - 12V LiPo battery powers motors and motor drivers.  
  - Power bank supplies Raspberry Pi, Teensy microcontroller, and IMU.  
- **Control System**:  
  - Raspberry Pi 4 handles high-level planning (ROS).  
  - Teensy manages low-level PID control for motors.  
- **Sensors**:  
  - Encoders provide wheel odometry feedback.
  - Encoders and IMU data are fused using EKF for localization. 
  - LiDAR is used for scan matching for AMCL. 
    

![Electrical Architecture](rootbot-mecanum-mobile-robot/images/electrical_system.png)  

---

## 🌍 Environmental Setup  
The testing environment simulates dynamic indoor scenarios with static and moving obstacles:  
- **Start Point**: Blue **X** marks the robot’s initial position.  
- **Goal Points**: Red locations represent target destinations for navigation tests.  
- **Obstacles**:  
  - **Dynamic** (yellow circles): Moving objects.  
  - **Static** (grey squares): Fixed barriers.  

![Test Environment](rootbot-mecanum-mobile-robot/images/Environment.png)  

---

## 📈 Performance Evaluation  
Planners were tested in real-world conditions across several metrics:  
- **Planning Time**: Fastest with Optimized RRT (~0.012s)  
- **Execution Time**: Best balance achieved by A* + DWA (~34–36s)  
- **Path Smoothness**: Higher with TEB, especially in narrow/dynamic areas  
- **Memory Usage**: Lowest with Optimized RRT (~62MB)  
- **Distance to Goal**: Shortest with Dijkstra (~10.4m), A* comparable (~11m)  

---

## 🧠 Algorithms Compared  
| Global Planner | Local Planner | Characteristics |  
|----------------|---------------|------------------|  
| Dijkstra       | DWA / TEB     | Optimal path, slower, high CPU |  
| A*             | DWA / TEB     | Heuristic-based, balanced |  
| RRT (Optimized)| DWA / TEB     | Fastest, responsive but less smooth |  
| Hybrid         | A* + DWA, RRT + TEB | Adaptive in dynamic environments |  

---

## 🧪 Simulation & Testing  
- Visualized mapping and localization with RViz.  
- Compared path planning algorithms in static and dynamic environments.  
- Analyzed performance on both mecanum and differential drives.  

---

## 📚 Documentation  
- Full thesis available in [`thesis/RBOT_RP4.pdf`](./thesis/RBOT_RP4.pdf).  
- Includes:  
  - Kinematic modeling  
  - Control design  
  - SLAM theory  
  - Planner pseudocode  
  - Experimental results and plots  

---

## 📌 Future Work  
- Integrate adaptive or AI-based navigation techniques.  
- Expand testing to 3D and outdoor environments.  
- Develop new hybrid path planners.  
- Real-time replanning under sensor failure conditions.  

---

## 👩‍💻 Author  
**Samaa Sayed Ahmed**  
Bachelor of Engineering in Robotics  
The British University in Egypt  
Supervised by: Dr. Abdalla ElGammal  

## 📜 License  
This project is for academic and research purposes. Contact the author for reuse or collaboration.  

--- 

