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



[images01]




# GraphSLAM 

GraphSLAM is SLAM algorithm that solves the FullSLAM problem. The algorithm recovers the **entire path and map** instead of just the most recent pose and map. This technique allows the robot to consider depenedencies betweencurrent and previous poses. Consider an example situation where a miner robot is informed of an accident in the mines. The most recent map that is stored in the robot's memory is likely to be no longer correct due to environmental changes. However, the robot is equipped with a LIDAR sensor so it travels around the environment collecting data about the surroundings. The data is then analyzed to create an accurate map of the environment. 

Benefits of this algorithm
---

1. Reduced need for significant onboard processing capability. 
2. Improve accuracy over FastSLAM

FastSLAM uses particles to estimate the robot's **most likely pose**. However, at any point in time, it's possible that there isn't a particle in the most likely location. The chances are even smaller in larger environments. 

Since GraphSLAM solves the fullSLAM problem, it can work with all of the data at once to find the optimal solution. FastSLAM uses minute amounts of information with a finite number of particles, to leave room for error. 

The following content will introduce:

1. The fundamentals of GraphSLAM 
2. How to construct Graph 
3. How to optimize within their constraints to create the most likely set of poses and map. 
4. How to overcome GraphSLAM's limitations

Graphs 
---

The GraphSLAM algorithm uses graphs to reprsent the robot's poses and the environment. We build a simple graph to further demonstrate how this workds. 
[image01]

A robot's poses (i.e position and orientation) can be represented by a node at time stamp zero:
[image002]


Usually the first node is arbitrarily constrained to zero, or it's equivalent in greater dimensions. The robot's pose at timestamp one can be represented by another node. The two would be connected by an edge, aka an arc. THis edge is a soft spatial constraint between the tow robot poses. These constraints are called soft because motion and measurenment data is uncertain. The constraints will have some amount of error present.

Soft constraint forms:

1. Motion constraints between two successive robot poses
2. Measurement constraints between a robt pose and a feature in the environment 

The constraints seen between x_0 and x_1 is a motion constraint. If the robot were to sense it's environment and encounter a feature m_1, a soft measurement constraint would be added.

[image003]

The star represents a feature in the environment. This could be s landmark specifically placed for the purpose of localization and mapping. Or it could be an identifiable element of the environment such as a corner or an edge. The notation chosen here has solid edge to reprsent motion contratints and dsahed edges to represent measurement constraints.

As we saw before, the robot poses are labeled x_1 and x_2 while the features will be labeled m_1 and m_2. As the robot moves around more and more nodes are added to the graph. Over time, the graph constructed by the moile robot becomes very large in size. The graphSLAM algorithm is capable of handling large numbers of features. 

Constraints 
--
It is convienent to interpert *soft constraints* as masses connected by rubber bands or springs. When no external forces are acting on them, the springs will naturally bring the system into a configuration, where the forces experienced by all of the springs are minimized.

When the nodes are connected in a linear function as below

[image004[or equation]

the resting configuration is easy to find but the process becomes more challenging as nodes become more and more interconnected. The springs will all try to push or pull the system in their own ways. 

This idea translates well to how constraints work in a graph. Every motion or measurement constraints pulls a system close to that constraint's desired state. Since the measurment and motions are uncertain, constraits will conflict with each other and there will always be some error present. 

The goal is to find the node configuration that minimizes the overall error present in all the constraints. 


Front-End vs Back-End 
--
The goal of GraphSLAM is to create a graph of all robot poses and features encountered in the environment and find the most likely robot path and map of the environment. 

This task can be broken up into two sections:

1. Front-end 
2. Back-end

Front-end
---
Looks at how to construct the graph using the odometry and sensory measurments collected by the robot. This includes:

1. Interperting sensory data
2. Creating the graph 
3. Continuing to add nodes and edges to the grpah as the robot traverses the environment.

The Front-end can be very different depending on application and deired goal

1. Accuracy
2. Sensor used 

The front-end of a mobile robot applying SLAM in an office using a laser range finder would differe greatly from the front-ednd for a vehicle operationg on a large outdoor environment and using a stereo camera. The front-end of GraphSLAM as the challenge of solving the data association problem. More specifically, this can be best described as accurately indentifying whether features in the environment have been previously seen. 

Back-end
---
The back-end is an optimization process that takes all of the constraints and find the system configuration tha produces the smallest error. The back-end is a lot more consistent across applications. The front-end and back-end can be completed in succession or can be performed iteratively, where a back-end is feeding an updated graph to the front-end for further processing. 

Maximum Likelihood Estimation 
--
At the core of GraphSLAM is *graph optimization* 

**graph optimization** - the process of minimizing the error present in all of the constraints in the graph. We'll study these constraints in depth and learn to apply a principle called *maximum likelihood estimation* (MLE) to structure and solve the optimization problem for the graph. 

Likelihood
---
Likelihood is a complementary principle to probability. Probability tries to estimate the outcomegiven the parameters, while likelihood tries to estimate **the parameters that best explain that outcome. 

When applied to SLAM, likelihood tries to estimate the most likely confiuration of state and feature locations given the motion and measurment observations. 

Feature Measurement Example: 
---
A robot is taking repeated measurments of a feature in the environment. The following example will walk us through the steps required to solve it, which can then be applied to more complicated problems. 

The robot's initial pose has a varianve of 0, becuase this is its start location. Recall, whereebver the start location may be, we call it *location 0* in our relative map. Every action pose and measurmenet hereafter will be uncertain. In GraphSLAM, we will make the assumption that motion and measurment data has Gaussian noise. 

The robot takes a measurment of its first feature, m1, and it returns a distance of 1.8 meters. 

If we return to the spring analogy, the 1.8m is the spring's resting length. This is the spring's most desirable length; however it is possible for the spring to be compressed or elongated to accomodate other forces contraints that are acting on the system. 

This probabilty distribution for this measurement can be defined as so: 

<a href="https://www.codecogs.com/eqnedit.php?latex=p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" title="p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" /></a>

Put more simply, the probability distribution is highest when z_1 and x_0 are 1.8 meters apart
[image005]

Since the location of the first pose x_0 is set to 0, this term can be removed from the equation:
<a href="https://www.codecogs.com/eqnedit.php?latex=p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" title="p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}" /></a>

Next, the robot takes another measurement of the same feature in the environment. This time, the data reads 2.2m. Now that there are two conflicting measurements the system is now considered **overdetermined** 

*overdetermined* - as system with more equations than unknowns. 

[image006]

With twoo measurements, the most probable location of the feature can be represented by the product of the two probabilities.

<a href="https://www.codecogs.com/eqnedit.php?latex=p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast&space;\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast&space;\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" title="p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast \frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" /></a>

In this simple example it is clear that most likely location of the feature is at the 2.0 meter mark. However, it is valuable to go through the maximum likelihood estimation process to understand the steps entailed, to be able to apply it to more complicated systems. 

To solve this problem analystically, a few steps can be taken to reduce the equations into a simplier form. 

Remove Scaling Factors
---

The value of m that maximizes the equation does not depend on the constants in front of each of the exponentials. These are scaling factors. In SLAM we are not primarily concerned witht the absolute value of the proabiliteis but finding the maximum likelihood estimate. Thus these factors can be removed. 

<a href="https://www.codecogs.com/eqnedit.php?latex=p(x)=e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast&space;e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(x)=e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast&space;e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" title="p(x)=e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}}\ast e^{-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" /></a>


Log-Likelihood
---
The product of the probabilities has been simplified so far, but the equation is still prety complicated witht the exponentials present. However, there is a mathematical property that can be applied to conver this product of exponentials into the sum of their exponents.

**Property**
<a href="https://www.codecogs.com/eqnedit.php?latex=e^ae^b=e^{(a&plus;b)}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e^ae^b=e^{(a&plus;b)}" title="e^ae^b=e^{(a+b)}" /></a>

**Results** 

<a href="https://www.codecogs.com/eqnedit.php?latex=e^{-\frac{1}{2}}\frac{(z_1&space;-&space;1.8)^2}{\sigma^2}&space;*&space;e^{-\frac{1}{2}}\frac{(z_1&space;-&space;2.2)^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e^{-\frac{1}{2}}\frac{(z_1&space;-&space;1.8)^2}{\sigma^2}&space;*&space;e^{-\frac{1}{2}}\frac{(z_1&space;-&space;2.2)^2}{\sigma^2}" title="e^{-\frac{1}{2}}\frac{(z_1 - 1.8)^2}{\sigma^2} * e^{-\frac{1}{2}}\frac{(z_1 - 2.2)^2}{\sigma^2}" /></a>


<a href="https://www.codecogs.com/eqnedit.php?latex=e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" title="e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" /></a>


Instead of working with the likelihood, we can take its natural logarithm and work that instead.

<a href="https://www.codecogs.com/eqnedit.php?latex=ln(e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?ln(e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}})" title="ln(e^{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}})" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex={-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" title="{-\frac{1}{2}\frac{(z_1-1.8)^2}{\sigma^2}-\frac{1}{2}\frac{(z_1-2.2)^2}{\sigma^2}}" /></a>

The natural logarithm is a monotonic function, it is always increasing as can be seen below:

[image007]

When working with logs of likelihoods, always expect a negativ evalue. This is due to the fact that proabbilities assume values between 0 and 1, and the log of any value between 0 and 1 is negative. THis can be seen in the graph above. When working with log-likelihoods, optimization entails minimizing the negative log-likelihood whereas in the past we were trying to maximize likelihood. 

Lastly, the constraints in front of the equation can be removed without consequence. We will assume for this example that the same sensor was used while obtaining both measurments, we can thus ignor the variance in the equation. 

<a href="https://www.codecogs.com/eqnedit.php?latex=(z_1-1.8)^2&plus;(z_1-2.2)^2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(z_1-1.8)^2&plus;(z_1-2.2)^2" title="(z_1-1.8)^2+(z_1-2.2)^2" /></a>

Optimization
---

The equation has been greatly reduced at this point. To get it to is simpliest formm, we multiply out all of the terms. 

<a href="https://www.codecogs.com/eqnedit.php?latex=2z_1^{2}-8z_1&plus;8.08" target="_blank"><img src="https://latex.codecogs.com/gif.latex?2z_1^{2}-8z_1&plus;8.08" title="2z_1^{2}-8z_1+8.08" /></a>

To find the minimum of this equation, we take the first derivative of the equation and set it to equal 0. 

<a href="https://www.codecogs.com/eqnedit.php?latex=4z_1&space;-8&space;=0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?4z_1&space;-8&space;=0" title="4z_1 -8 =0" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=4z_1&space;=8" target="_blank"><img src="https://latex.codecogs.com/gif.latex?4z_1&space;=8" title="4z_1 =8" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=z_1&space;=2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?z_1&space;=2" title="z_1 =2" /></a>

We are finding the location on the curve where the slope (or gradient, in multi-dimensional equations) is equal to zero i.e the trough.

The property can be visualized easily by looking at a graph of the error function. 
[image008]

In more complex examples, the curve may be multimodal, or exist over a greater number of dimensions. If the curve is multimodal, it may be unclear whether the locations discovered by the first derivative are in fact troughs, or peaks. In such a cas, the second derivative of the function can be taken, which should clarify whether the local feature is a local minimum or maximum. 

Overview
---

The procedure we executed here is the *analytical* solution to an MLE problem. The steps include, 

* Removing inconsequential constants 
* Converting the equation from one of lieklihood estimation to one of *negative log-likelihood estimation*
* Calculating the first derivative of the function and setting equal to zero to find the extrema. 

In GraphSLAM, the first two steps can be applied to *every* constraint. Thus any measurement or motion constraint can besimmply labelled with is negative log-likelihood error. For a measurment constraint, this would resemble the following:

<a href="https://www.codecogs.com/eqnedit.php?latex=\frac{(z_t-(x_t&plus;m_t))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{(z_t-(x_t&plus;m_t))^2}{\sigma^2}" title="\frac{(z_t-(x_t+m_t))^2}{\sigma^2}" /></a>

For the motion constraint:

<a href="https://www.codecogs.com/eqnedit.php?latex=\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}" title="\frac{(x_t-(x_{t-1}+u_t))^2}{\sigma^2}" /></a>

From now on, constraints will be labelled with their negative log-likelihood error, with the estimation function trying to minimize the sum of all constraints.

[image009] 


<a href="https://www.codecogs.com/eqnedit.php?latex=J_{GraphSLAM}=\sum_{t}\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}&space;&plus;\sum_{t}\frac{(z_t-(x_{t}&plus;m_t))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?J_{GraphSLAM}=\sum_{t}\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}&space;&plus;\sum_{t}\frac{(z_t-(x_{t}&plus;m_t))^2}{\sigma^2}" title="J_{GraphSLAM}=\sum_{t}\frac{(x_t-(x_{t-1}+u_t))^2}{\sigma^2} +\sum_{t}\frac{(z_t-(x_{t}+m_t))^2}{\sigma^2}" /></a>

MLE Example 
--

In the previous example we looked at a robot taking repeated measurment of the same feature in the environment. This example demonstrates the fundamentals of maximum likelihood estimation, bbut was extremly limited since it was only estimating one parameter - *z_1*

In this example, we have the opportunity to get hands-on with a more complicated 1-dimensional estimation problem. 

Motion and Measurement Example
---
The robot starts at an arbitrary location that will be labeled 0, and then proceeds to measure a feature in front of it - the sensor reads that the feature is 7 meters way. The resultant graph is shown in the image below. 

[image010]

After taking its first measurement, the following Gaussian distribution describe the robot's mostly likely location. The distribution is highest when the two poses are 3 meters apart. 

<a href="https://www.codecogs.com/eqnedit.php?latex=p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-7)^2}{\sigma^2}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-7)^2}{\sigma^2}}" title="p(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{1}{2}\frac{(z_1-7)^2}{\sigma^2}}" /></a>

Recall that since we constrained the robot's initial location to 0, x_0 can actually be removed from the equation. 

Next, the robot moves forward by what it records to be 10 meters, and takes another measurement of the same feature. This time, the feature is read to be 4 meters behind the robot. The resultant graph looks like this:

[image011]


[image012] 


Sum of Constraints
---

The completed graph, with all of its labelled constraints can be seen as follows:

[image013]

The task is to now minimize the sum of all constraints:





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
