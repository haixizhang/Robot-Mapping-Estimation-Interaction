# Haixi Zhang's final project for ECE218 

## Academic Honesty
I affirm that I have not given or receive any unauthorized help on this exercise, and that all work is my own. 

## Src packages Lists
Launch, Gui, Simulator, Perception, Mapper, Localization, Navigator, Executive
* In GUI, the real odom is painted in color of black and the estimated odom is painted in color of red.
* In GUI, the goal is set to be a darker red, and the navigation trajectory is set to be lighter blue. 
* I place the Apriltags package outside the src folder.
* Some important topic names: executive/waypoints, mobile_base/commands/velocity, sampled_trajectories, 
## Launch Files 
* Launch files are listed in the package of /src/launch folder. 
* The **amr_sim.launch** file is used for simulation, and the **robot.launch** is used for running on robot.
* Currently I did not comment the landmarks publisher and the waypoints publisher in the robot.launch file. 
 

## Points Publisher
* The **Landmarks publisher** file is in the path of ~/src/simulator/src/landmarks_publisher.cpp, and currectly this file include the same landmarks as the Blackboard Landmarks Publisher Example version.

* The **Waypoints publisher** file is in the path of ~/src/executive/src/waypoints_publisher.cpp, and currectly this file include the same wavepoints as the Blackboard Waypoints Publisher Example version.

## Expectations
* The simulation should work perfectly
* For running on the robot, I have set the maximum speed of the robot to be 0.3m/s, and solve the problem that robot behaves abnormally rotation in the GUI. 

