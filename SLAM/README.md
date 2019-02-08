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
--
Previously, we discussed the *discrete* and *continuous* nature of SLAM:

* Discrete: This component deals with **correspondence** therefore we will spend some time discussing it in detail. In our estimation problem, the posterior should now include the corresondence values in both the online and Full (offline) SLAM problems. 

When we solve the online SLAM problem, our goal is to estimate a posterior over the current pose, map and the current correspondence value, **c_t** gien the measurement **z_t** and control signal **u_t** up to time **t**.

Alternatively, if we want to solve the Full SLAM problem, our goal is to estimate a posterior over the entire path, map and all of the correspondence values, **c_t** up to time **t**, given **all** the measurements **z_t** and control signals **u_t** up to time **t**. 

Since we have add the correspondence values to both the posterior of the online and full SLAM, our goal is to:

1. Estimate the map
2. Estimate the robot pose or trajectory

Adding correspondence values to the posterior has its advantages. For starters, the robot can better unstand where it is located by establishing a relation between objects. With the addition of correspondence values, the online SLAM problem is now the result of integrating the robot poses once at a time and summing over each previous correspondence value. 

SLAM  Challenges
--

Computing the full posterior composed of the robot pose, the map and the correspondence under SLAM poses a big chalenge in robotics mainly due to the continous and discrete natures

[image1] 

### Continuous

The continuous paramter space is composed of the robot poses and the location of objects is high dimensional. When mapping the environment and localizing itself, the robot will surely encounter many objects and will have to keep track of each one of them. Therefore, the number of variables will increase with time. 

### Discrete

The discrete parameter space is composed of the correspondence values, and is also highly dimensional due to the large number of correspondence variables. The correspondence values will also increase exponentially over time since the robot will keep sensing the environement and relationg the newly detected objecs to the previously detected ones. 

Overall, it is not feasible to compute the posterior under unknown correspondence. Therefore, SLAM algorithms will have to rely on approximation while estimating a posteriorr. Doing so will conserve computational memory. 

Particle Filter Approach to SLAM 
--
The challenges we presented above can be overcome with the techniques introduced in this section. Let's recap what we learned from localization. When we solve the localization problem, we use different approaches to estimate a robot's pose inside an environment. Previously we looked at the **particle filter** approach implemented in the Monte Carlo localization algorithm. Each of the particles spawned, will have the robot pose along with the **importance** weight of the particle.  Using this approach, we were able to accurately estimate any robot's pose. 

Moving forward we will imagine a modified particle filter aproach where we add another dimension to each particle. The new particle will now how the robot's pose and the map. Although this does not solve the SLAM problem to the map being modeled with many variables; which results in high dimensionality. The particle filter approach to SLAM in this form will inevitably fail in the field. 

NOTE: By adding another dimension to a particle will allow it hold the robot's pose, wight and the map and then solving through MCL in its current form will fail. 



Introduction to FastSLAM 
--

The FastSLAM algorithm uses a customized particle filter approach to solve the full SLAM problem with known correspondences. Using particles, FastSLAM estiamtes a posterior over the robot's path along with the map. 

Each of the particles hold the robot's trajectory which will give an advantge to SLA to solve the problem of mapping with known poses. In addition to the robot's trajectory, each particle holds a map and each feature of the map is represented by a **local Gaussian**. With the FastSLAM algorithm, the problemm is divided into separate independent problems. These problems aim to solve the problem of estimating feaures of the map. To solve these independent mini problems, FastSLAM will use the low dimensional **Extended Kalman Filter**. 

In this approach, math features are treated independently, and dependency only exists between the robot pose uncertainity. The aforementioned approach that involves representing the posterior with particle filters and Gaussians is know as the Rao-Blackwellized Particle Filter. 

* **Estimating the Trajectory**: FastSLAM estimates a posterior over the trajectory using a particle filter approach. This will allow SLAM to solve the problem of mapping with known poses. 
* **Estimate the Map** FastSLAM uses a low dimensional Extended Kalman Filter to solve indpenent features of the map whcih are modeled with the local Gaussian. 

FastSLAM Instances 
--

Since FastSLAM uses a particle filter approach to solve SLAM problems, some robotixists consider it a powerful algorithm capable of solving both the Full SLAM and Online SLAM problems. 

* FastSLAM estimates the full robot path.
* Each particle in FastSLAM estimates instantaneous poses and thus FastSLAM also solves the Online SLAM problem.

There are 3 different instances of the FastSLAM algorithm:

**FastSLAM 1.0**

This algorithm is simple and easy to implement, but is known to be inefficient since particle filters generate sample inefficiency. 

**FastSLAM 2.0** 

This algorithm overcomes the inefficiency of FastSLAM 1.0 by imposing a different distribution, which results in a low number of particles. Both 1.0 and 2.0 algorithms use a low dimensional Extended Kalman Filter to estimate the posterior over the map features. 

**Grid-based FastSLAM** 

Lastly, this algorithm adapts FastSLAM to grid maps. 

Adapting FastSLAM to Grid Maps
--

The advantage of the FastSLAM algorithm is that it uses the particle filter approach to solve the SLAM problem. Each particle will hold a guess of the robot trajectory, and by doing so, the SLAM problem is reduced to mapping with known poses. However, this algorithm presents a disadvantage since it must always assume that there are known landmark positions, and thus with FastSLAM we are not able to model an arbitary environment. Even if the landmark positions are unavailable to use, we are still able to use 

Grid-based FastSLAM Technique 
--
With the grid mapping algorthim can model the environment using grid maps without predefining any landmark position. By the extending the FastSLAM algorithm to occupancy grid maps, we can now solve the SLAM problem in an arbitrary environment. During the process of mapping a real-world envirnment, we are likely to be usign a mobile robot equipped with range sensors. We can therefore extend the FastSLAM algorithm and solve the SLAM problem in terms of grid maps.

Robot Trajectory
--
Similarly to the FastSLAM algorithm, the particles in the grid-based FastSLAM algorithm holds a guess of the robot's trajectory.

Map
--
Each particle also maintains its own map. The grid-based FastSLAM algorithm will update each particle by solving the mapping with known poses problem using the occupancy grid mapping algorithm. 

Grid-based FastSLAM techniques
--
Since the Grid-based FastSLAM algorithm uses a particle filter approach and represents the world in terms of grid maps will need to combine our knowledge of **MCL** and the **Occupancy Grid Mapping** Algorithm. 

We need **3** different techniques, represented by **3** probability functions to adapt FastSLAM to grid mapping.

1. Sampling motion
2. Map estimation 
3. Importance weight

Sampling Motion
--
We use this technique to estimate the current pose giving the K-th particle's **previous pose** and **the current control** **u**

Map Estimation
---
The objective is to estimate the curent map given the **current measurments**, **the current K-th particle's pose**, and **the previous K-th particle's map**

If we examine it's probability function, we can relate it to the mapping problem in which our goal is to estimate the map given the robot's trajectory. 

<a href="https://www.codecogs.com/eqnedit.php?latex=p(m_t|z_t,x_{t-1}^{k},&space;u_t)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(m_t|z_t,x_{t-1}^{k},&space;u_t)" title="p(m_t|z_t,x_{t-1}^{k}, u_t)" /></a>

To solve the map estimation problem, we will make use of the Occupany **Grid Mapping Algorithm.**

Importance Weight
--
This algorithm computes the *importance weight* of each individual particle. It estimates the current likelihood of the measurement given the current K-th particle pose and the current K-th particle map. 

The sampling motion and importance weight will be both solved with the MCL algorithm whereas the map estimation technique will be solved with the occupancy grid mapping algorithm.

The Grid-based FastSLAM algorithm 
---

*Sampling motion*, *map estimation* and *importance weight* techniques are the foundations of the Grid-based FastSLAM algorithm. Grid-based FastSLAM implements them to estimate both the map and the robot trajectory given the measurements and the control. 

The Grid-based FastSLAM algorithm looks very similar to the MCL algorithm with additional statements that include the map estimation. The algorithm itself, is a the result of combining the MCL algorithm and the occupancy grid mappiping into one. 

In the MCL algorithm, the Grid0based FastSLAM algorithm is compised of two sections represented by two *for* loops. The first section includes the motion, sensor, and map update steps. The secod one includes the re-sampling process. At each iteration the algorithm takes:

* The previous belief or pose
* the actuation command
* The sensor measurements

As inputs. Initially, the hypothetical belief is obtained by randomly generating M particles. 

Next, in the first section, each particle implements the three techniques covered earlier to estimate the K-th particle's current pose, likelihood of the measurement and the map. 

Each particle begins by implementing the sampling technique in the ``sample_motion_model()`` function to estimate the current pose of the K-th particle. 

Moving forward, in the measurement update step, each particle implements the importance weight technique in the ``measurement_model_map()`` function to estimate the current liklihood of the K-th particle measurement. 

Next, in the map update step, each particle will implement the map estimation technique into the ``update_occupancy_grid()`` map function to estimate the current k particle map. This map estimation problme will be solved using the occupancy grid mapping algorithm. 

Finally, the newly estimated K particle pose map and likelihood of the measurments are all added to the hypothetical belief. 

In the second section of the algorithmm the resampling process happens through a **re-sampling wheel** Particles with measurement values close to the robot's measurment value survive and are redrawn in the next iteration, while the others will die. 

Surviving particle poses and map are then added to the system's belief. 

Lastly, the algorithm outputs the new belief and another cycle of iteration starts implemeneting the newly computed belief, the next motion and the new sensor measurements. 

The algorithm is summarized below: 

* Step 1: Previous Belief
* Step 2: Sampling motion
* Step 3: Importance weight
* Step 4: Map Estimation 
* Step 5: Resampling 
* Step 6: New Belief



gmapping ROS Package
---

We will implement the ``gmapping`` ROS package which is based on the Grid-based FastSLAM algorthim to map an environment.

``gmapping`` provides laser based SLAM. This means that we can feed its node with the robot laser measurements and odometry values and expect it to provide you with a 2D occupancy grid map of the environment. That map will be updated as robot moves and collect sensory information using its laser range finder sensor.

``gmapping`` package documentaion

Use this [link](http://wiki.ros.org/gmapping) to review the ``gmapping`` ROS package. 

Deploying a Turtlebot in a Willow Garage environment
---

Create a ``catkin_ws`` in ``/home/workspace/`` 
 --
 ``` shell 
$ mkdir -p /home/workspace/catkin_ws/src
$ cd /home/workspace/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```
Perform a system Update/Upgrade

```shell 
$ apt-get update
$ apt-get upgrade -y

```

Clone ``turtlebot_gazebo``and ``turtlebot_teleop`` in src
---
```shell
$ cd src/
$ git clone https://github.com/turtlebot/turtlebot_simulator
$ git clone https://github.com/turtlebot/turtlebot
```

Install package dependencies
---
``shell
$ cd ..
$ source devel/setup.bash
$ rosdep -i install turtlebot_gazebo
$ rosdep -i install turtlebot_teleop
``
Buuilding the packages
--
``shell
$ catkin_make
$ source devel/setup.bash
``

SLAM with ROS
---



# GraphSLAM 


Graphs 
--

Constraints 
--

Front-End vs Back-End 
--

Maximum Likelihood Estimation 
--

MLE Example 
--

Numerical Solution to MLE 
--

Mid-Lesson Overview 
--

1-D to n-D 
--

Information Matrix and Vector 
--

Inference 
--

Nonlinear Constraints 
--

Graph-SLAM at a Glaance 
--

Intro to 3D SLAM with RTAB-Map 
--

3D SLAM with RTAB-Map 
--

Visual Bag-of-Words 
--

RTAB-Map Memory Management 
--

RTAB-Map Optimization and Output 
--
