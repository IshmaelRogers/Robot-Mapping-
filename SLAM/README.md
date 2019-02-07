# Simultaneous Localization and Mapping 
# Ishmael Rogers
# Robotics Software Engineer
# Infinitely Deep Robotics Group 
# 2019

# Grid-based FastSLAM

ImuSLAM is the process of estimating both the map and the robot poses in real-world environments. In mapping problems we are provided with the exact robot poses and then we estimate the map of the environment. In this repo we will combine our knowledfe from both localization and mapping to solve the most fundamental problem in robotics. SLAM involves mapping an environment, given noisy measurements and localizing the robot relative to its own map given the controls. 

This problem is extremely difficult because both the map and the poses are unknown. Real-world environments will necessitate the need for SLAM. The robot vaccum clean uses the measurements provided by its laser finder sensor and data from the encoders to estimate the map and localize itself relative to it.


Online SLAM 
--

Given measurements, z and the control signal, u, we can estimate calculate the posterior map, m, as well as the poses, x. The following two forms of SLAM are very important to robotics:

1. *Online SLAM*
2. Full SLAM

* At time **t-1**, the robot will estimate its current pose, **x_t-1** and the map, **m** given its current measurements **z_t-1** and controls **u_t-1**. At time **t**, the robot will estimate its new pose, **x_t**, and the map, **m** given only its current measurements z_t and controls u_t. 

NOTE: The previous measurements and controls are not taken into consideration when computing the new estimate of the pose and map. 

* At time, **t+1** the robot will estimate its current pose **x_t+1** and the map, **m** given the measurements **z_t+1** and controls **u_t+1**. 

With the Online SLAM problem we will solve instantaneous poses indepedentrly from previous measurements and controls. We are able to model this problem with the probability equation P(x_t,m|z1:t, u1:t). We can solve for the posterior represented by the instantaneous pose and the map given the measurments and controls. 

In general, with online SLAM we will only estimate variables that occur at time **t** only. 


Full SLAM
---

In this problem we will estimate the entire path up to time t, instead of an instantaneous posee given all the measurements and controls 

* At time t-1, the robot will estimate the robot pose x_t-1 and map m, given the measurements z_t-1 and controls u_t-1.
* At time t, the robot will estimate the entire path x_t-1:t and map m, given all the measurements z_t-1:t and controls u_t-1:t.
* At time t+1, the robot will estimate the entire path x_t-1:t+1 and map m, given all the measurements z_t-1:t+1 and controls u_t-1:t+1.


This problem can be modeled with the probability equation p(x_1:t , m | z_1:t u_1:t), where we solve the posterior represented by the robot's trajectory x_1:t and the map m given **all** the measurements z_1:t and controls u_1:t. 


In general, in the Full SLAM problem we will estimate all the variables that occur throughout the robot travel time.

Online vs Offline
-----

Offline represents the posterior over the entire path.

Online respresents the posterior over the current pose. 

Integrating the previous poses from the Full SLAM problem, one at a time, we can obtain the Online SLAM problem 

Nature of SLAM
--
SLAM problems have a continuous and discrete nature:

### Continuous Nature
During SLAM, a robot continuously collects odometry information to estimate the robot poses and continuously
Correspondence


SLAM  Challenges


Particle Filter Approach to SLAM 

Introduction to FastSLAM 

FastSLAM Instances 

Adapting FastSLAM to Grid Maps

Grid-based FastSLAM Technique 

The Grid-based FastSLAM algorithm 

gmapping ROS Package

SLAM with ROS



# GraphSLAM 


# Graphs 

# Constraints 

# Front-End vs Back-End 

# Maximum Likelihood Estimation 

# MLE Example 

# Numerical Solution to MLE 

# Mid-Lesson Overview 

# 1-D to n-D 


# Information Matrix and Vector 

# Inference 


# Nonlinear Constraints 

# Graph-SLAM at a Glaance 


# Intro to 3D SLAM with RTAB-Map 


# 3D SLAM with RTAB-Map 


# Visual Bag-of-Words 


# RTAB-Map Memory Management 


# RTAB-Map Optimization and Output 

