# Mobile Robot Mapping, Estimation, and Interaction 

## Objective:
Developed and tested a ROS-based software architecture to enable autonomous navigation and exploration in a partially known or unknown environment using a TurtleBot2 mobile robot and simulator.

## Overview: 
Developed an advanced autonomous navigation system integrating input space sampling, Extended Kalman Filter (EKF) SLAM, and real-time perception using ROS and C++.

## Key Components:
1. Navigation
* Input Space Sampling: Trajectory Generation & Sampling: Created and sampled trajectories based on dynamic models, ensuring obstacle-free paths using collision checking and cost-based optimization.
* Motion Planning: Implemented a cost-function to evaluate trajectories, considering goal proximity, obstacle avoidance, and velocity constraints.

2. Perception System:
* AprilTags Detection: Leveraged fiducial markers for environmental awareness, extracting and processing RGB images to detect tags and estimate their poses.
* Depth Image Processing: Analyzed depth images for 3D structural understanding and mapping.

3. Occupancy Grid Mapping System:
* Grid-Based Mapping: Maintained a probabilistic occupancy grid, updated in real-time with laser scan data to represent the environment accurately.
* Log-Odds Representation: Applied log-odds updates for cells based on sensor observations, balancing between free and occupied states.

4. Localization:
* EKF Implementation: Employed an Extended Kalman Filter for precise state estimation, integrating motion predictions and sensor measurements.
* Covariance Management: Managed uncertainty through covariance updates, enhancing localization robustness.

5. Simulation and GUI: Designed a graphical user interface to visualize occupancy grids, planned paths, laser scans, goals, sampled trajectories, and observed landmarks. Developed simulation capabilities to emulate robot motion and sensor observations.

## Src packages Lists
Launch, Gui, Simulator, Perception, Mapper, Localization, Navigator, Executive
* In GUI, the real odom is painted in color of black and the estimated odom is painted in color of red.
* In GUI, the goal is set to be a darker red, and the navigation trajectory is set to be lighter blue. 
* Some important topic names: executive/waypoints, mobile_base/commands/velocity, sampled_trajectories.
* Install a third-party library to main ros directory by using the command :
  git clone https://bitbucket.org/kaess/apriltags.git
* Use catkin_make command to build the project
  
## Launch Files 
* Launch files are listed in the package of /src/launch folder. 
* The **amr_sim.launch** file is used for simulation, and the **robot.launch** is used for running on robot.
* Currently I did not comment the landmarks publisher and the waypoints publisher in the robot.launch file. 
 

## Points Publisher
* The **Landmarks publisher** file is in the path of ~/src/simulator/src/landmarks_publisher.cpp, feel free to change the x,y coordinates.

* The **Waypoints publisher** file is in the path of ~/src/executive/src/waypoints_publisher.cpp, feel free to change the x,y coordinates.

## Expectations
* The simulation and robot test both work perfectly.
* For running on the robot, I have set the maximum speed of the robot to be 0.3m/s to keep robot safe.

##
Acknowledgment: Special thanks to Professor Thomas M. Howard, Associate Professor of Electrical and Computer Engineering at the University of Rochester, for his guidance and support throughout the project.
