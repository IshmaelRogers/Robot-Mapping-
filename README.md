# Robot Mapping 
# Ishmael Rogers
# Infinitely Deep Robotics Group
# 2019

# Introduction 

Localization - A robot is provided with a known map as well as sensor and odometry data to estimate its pose in an environment. The pose is best decribed as the robot's x,y coordinates and orientation, [theta]. In the localization problem, there are only a finite number of varibales that we can use to decribe the x,y,[theta]. There are many methods to providing the robot with localization capabilities [link to localization repo]

Mapping - Insteading of assuming a known map and finding the robot's pose, we assume a known path and estimating the environment. The challenges associated with this problem lie in the fact that maps exist in a continuous space therefore there are infinitely many variables that can be used to describe it. Uncertainities in perception also contribute to this challenge. The nature of the space being mapped also contribute to the challenge of mapping. For example, the geometry and whether the space is repettive i.e does the space contain many different places that look alike. Imagine driving through a forest where the trees are all the same and are planted neatly in a row, it will be difficult to determine your location. In this repo, we will learn how to map an environment using the occupancy grid algorithm. Using this algorthim, it is possible to divide any arbitrary environment by dividing it into a finite number of grid cells. Estimating the sets of each individual cell, we can create an estimated map of the environment.           

SLAM - Simultaneous Localization and Mapping the robot must construct a map of the environemnt, while simultaneously localizing itself relative to this map. In this problem, the robot is not provided with a map nor its pose. Noisey motion and sensory data, make the robot's pose and map uncertain and the errors in these will be correlated. The accuracy of the map depends on the accuracy off the loclizarion and vice versa. There are five categories of SLAM algorthims:

1. Extended Kalman Filter SLAM 
2. Sparse Extended Informaion Filter 
3. Extended Information Form 
4. **FastSLAM** 
5. **GraphSLAM** 

In this repo we learn about the FastSLAM algorithm which is basically a particle filter approach with a low dimensional extended kalman filter to solve the SLAM problem. Next we'll adapt the algorithm to grid maps, which will result in a grid based FastSLAM algorithm.

Next, we'll learn about GraphSLAM. This algorthim uses constraints to represent relationships between the robot's pose and the environment. Next, the algorithm resolves these constraints to create the most likely map given the data. The real-time appearance based mapping or RTAB.

