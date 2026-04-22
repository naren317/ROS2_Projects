# ROS2_Projects
## Contents:
### 1.) Basic SLAM: Odometry and LiDAR data from a robot (tested with basic turtlebot3 burger) translated into a simple map. This map can be observed in rviz.

<img width="1145" height="512" alt="image" src="https://github.com/user-attachments/assets/c1b2091e-b996-4165-8901-72cb0e85b018" />
Basic turtlebot3 in Gazebo.

<img width="957" height="541" alt="image" src="https://github.com/user-attachments/assets/3ce9ff9f-4ce0-44dd-bc87-4fc3104b5c3c" /><br>
Map created with LiDAR data for a basic 2D SLAM using occupancy grid, visualization in rviz.

### 2.) Simple Pointcloud processing: A simple demo of point cloud processing, where in a point cloud is obtained by a gazebo simulated `depth_camera` and processed via a simple subscribing node using `pcl::PoinCloud`.

### 3.) Simple Interface: Interface created with custom data for publishing a timepoint and conversion to a localtime by the subscriber.


