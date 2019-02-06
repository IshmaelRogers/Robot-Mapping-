# Mapping 
# Ishmael Rogers
# Infinitely Deep Robotics Group 
# 2019 

# Importance of Mapping 


# Challenges and Difficulties 
 There are 2 main challenges associated wth mapping with mobile robots.
 
The map and poses are both unknown to the robot
---
In the localization section we assumed a known map and our goal was to estimate the robot's pose. In this section, the mape is uknown to us. Therefore, we'll have to assume either:
* Known poses and estimate the map
* Unknown poses and estimate the mapa and the poses relative to it. 

Estimating the map with unknown poses is challenging because of the large number of variables. We can use the *Occupancy Grid Algorithm* 

The hypothesis space is huge
---

*hypothesis space* - the space of all possible maps that can be formed during mapping. 

The hypothesis space highly dimensional because maps are defined over a continuous space. When robots are depolyed in open environments they must sense an infinite number of objects. The Occupancy Grid Mapping Algorithm is a discrete approximation of the map. Despite this discrete approximation, the space of all possible maps is still prett high. 

Overall, the challenge is to estimate the full posterior map, for maps with high dimensional space. The *Bayes Filterin* approach used in localization to estimate the posterior pose will divera and an extension to it should be used in mapping to accommodate for the huge hypothesis space. 

The difficulties in mapping
---

In order to map an environment we need information about the walls and objects in the map. For example, if we deploy a mobile robot with a laser range finder sensor, the robot will collect sensory information that allow it to detect obstacles in an environment. Using a mapping algorithm, we group this data into a resulting map. 

There are several possible data points to be represented by instantaneous maps that the robot needs to combine in order to estimate the actual map. 

1. The size of the environment - Mapping large areas is difficult because of the large amounts of data that needs to be processed. The robot's on-board micro-controller needs to be able collect all the instantaneous poses and obstacles and then form a resulting map and localize the robot with respect to this map. Mapping becomesmore of a challenge when the size of the map is larger than the robot's perceptual range. Furthermore, noise is always present in perception sensors such as odometry sensors and actuators. During the mapping process we must carefully filter the noise from these sensors and actuators. The larger the noise the more difficult mapping problem is.

2. Perceptual ambiguity - Ambiguity occrs when two places look alike and the robot must correlate between these two placew which the robot must travel through at different points in time. 

3. A robot traveling in a cyclic manner - when traveling in cycles the robot odometry incrementally accumulates error. At the end of the cycle the error value is usually very large. 

# Mapping with Known Poses 

Mapping with known poses is the process of generating a map under the assumption that the robot poses are known and non-noisy. The mapping with known poses problem can be represented with a graph as shown below. The x represent the poses, zrepresents the measurements and m represents the map. 

The Occupancy Grid Mapping Algorithm can estimate the posterior map given noisey measurements and known poses. In most robotics applications, odometry data is noisy and the robot's poses are unknown. 

Mapping usually occurs after SLAM. The usefulness of mapping is in its post-processing. In SLAM, the problem changes from mapping with known poses to mapping with unknown poes. During SLAM, the robot's build a map of the enivronment and localizes itself relative to it. 

After SLAM, the occupancy grid mapping algorithm uses the exact robot poses filtered from SLAM. With the known poses from SLAM and noisy measurements, the mapping algorithm generates a map fit for path planning and navigation. 

# Posterior Probability 

In the graphical model of mapping with known poses, our goal is to implement a mapping algorithm and estimate the map given noisy measurements and assuming known poses.

The Mapping with Known Poses problem can be represented with P(m | z_{1:t}, x_{1:t}) function. With this function, we can compute the posterior over the map given all the measurements up to time t and all the poses up to time t represented by the robot trajectory.

In estimating the map, we’ll exclude the controls u since the robot path is provided to us from SLAM. However, keep in mind that the robot controls will be included later in SLAM to estimate the robot’s trajectory.

2D Maps
For now, we will only estimate the posterior for two-dimensional maps. In the real world, a mobile robot with a two-dimensional laser rangefinder sensor is generally deployed on a flat surface to capture a slice of the 3D world. Those two-dimensional slices will be merged at each instant and partitioned into grid cells to estimate the posterior through the occupancy grid mapping algorithm. Three-dimensional maps can also be estimated through the occupancy grid algorithm, but at much higher computational memory because of the large number of noisy three-dimensional measurements that need to be filtered out.


# Grid Cells
In order to estimate the posterior map, the occupancy grid will uniformly partition the 2D space in a finite number of grid cells. Each of these grid cells will hold the binary random value that corresponds to the location it covers. Based on the measurements data, this grid space will be filled with zeros and ones. If the laser ranger finder sensor detects an obstacle, the cell will be consider as occupied and its value will be *1*. In the free spaces, the cell be considered unoccupied and its value will be *0*. 

To find out the number of maps that can be generated out of grid cells, we'll pick the first two cells below and work out their combinations:

[image2]

The first combination is a map with two free spaces. Next, the second combination is a map with one free space on the left side and one obstacle on the right side.Proceeding, the second combination is a mpa with one free space on the left side and one obstacle on the right side. Next, when we flip the second map, we get the third combination. Finally, the fourth combination is a map with two obstacles. In conclusion, the total number of maps that can be formed out of these two grid cells is eqal to four. To generalize, the numbe of maps that can be formed out of grid cells is equal to 2^n where in is the number of cells. 

If we imagine a 2D space with tens of thousands of grid cells the number of maps generated will be even larger!

# Computing the posterior 

First\ Approach: P(m | z_{1:t}, x_{1:t})

We just saw that maps have high dimensionality so it will be too pricey in terms of computational memory to compute the posterior under this first approach.

Second\ Approach:P(m_{i} | z_{1:t}, x_{1:t})

A second or better approach to estimating the posterior map is to decompose this problem into many separate problems. In each of these problems, we will compute the posterior map mi at each instant. However, this approach still presents some drawbacks because we are computing the probability of each cell independently. Thus, we still need to find a different approach that addresses the dependencies between neighboring cells.

Third\ Approach:\prod_{i} P( m_{i}| z_{1:t} , x_{1:t})

Now, the third approach is the best approach to computing the posterior map by relating cells and overcoming the huge computational memory, is to estimate the map with the product of marginals or factorization.

# Filtering 



# Binary Bayes Filter Algorithm 


# Occupancy Grip Mapping Algorithm 

# Inverse Sensor Model 


# Generate the Map 

# Multi-sensor Fusion 


# 3D Mapping 


# 3D Data Representations 


# Octomap 
