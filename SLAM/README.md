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

<a href="https://www.codecogs.com/eqnedit.php?latex=J_{GraphSLAM}=\frac{(z_1-7)^2}{\sigma^2}&space;&plus;&space;\frac{(x_1-(x_{0}&plus;10)^2}{\sigma^2}&space;&plus;&space;\frac{(z_1-(x_{1}-4))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?J_{GraphSLAM}=\frac{(z_1-7)^2}{\sigma^2}&space;&plus;&space;\frac{(x_1-(x_{0}&plus;10)^2}{\sigma^2}&space;&plus;&space;\frac{(z_1-(x_{1}-4))^2}{\sigma^2}" title="J_{GraphSLAM}=\frac{(z_1-7)^2}{\sigma^2} + \frac{(x_1-(x_{0}+10)^2}{\sigma^2} + \frac{(z_1-(x_{1}-4))^2}{\sigma^2}" /></a>


To do this, we will need to take the first derivative of the function and set it equal to zero. Because there are two variables, *Partial Derivatives* 

Optimization with Non-Trivial Variances
---

No we'll take into consideration the variances of each measurement and motion. Our robot has the fanciest wheels on the market - they’re solid rubber (they won’t deflate at different rates) - with the most expensive encoders. But, it looks like the funds ran dry after the purchase of the wheels - the sensor is of very poor quality.

We can redo the math with the following variances

Motion variance: 0.02,
Measurement variance: 0.1

z_1 = 6.54 
x_1 = 10.09



At this point we've only seen 3 constraints this process would be more difficult if had collected measurement and motion data over a period of half-an hour. This is more typical for a real-life environment. Solving the system analytically has the advantage of finding the *correct andswer*. In doing so, we require a lot of computational resources, espeically as we transition to multi-dimensional problems with complex probability distributions. Solcing the problem numerically allows for a solution to be found quickly, however its accuracy may be sub-optimal. 

Numerical Solution to MLE 
--

The method that we applied in the previous two examples was very effective at finding solutions quickly. In more complicated problems, finding the analytical solution may involved lengthy computations. The numerical solutions to maximum likelihood problems can be found in a fraction of the time. 

Numerical Solution
---

The graph of the error function from the previous example is seen below. In this example it is very easy to see where the global minimum is located. In a more complicated example with multiple dimensions, this is not as easy. 

[image012]

This Minimum Likelihood Estimation can be solved numerically by applying an optimization algorithm. The goal of an optimiziation algorithm is to speedily find the optimal solution, in this case, the local minimum. There are several different algorithms that can tackle this problem in SLAM:

* Gradient Descent
* Levenerg-Marquardt
* Conjugate gradient 

Graident Descent 

The gradient of a function is a vector that points in the direction of the greatest rate of change; or in the case on an extrema, is equal to zero. 

In gradient descent, we make an initial guess and then adjust it incrementally in the direction opposite the gradient. Eventually, we will reach a minimum of the function. 

This algorithm is not always perfect. In complex distribution the initial guess can change the end result significantly. Depending on the intiial guess the algorithm converges on two different local minima. The algorithm has no way to determine where the global minimum is, it naively moves down the steepest slope and when it reaches a local minima, it considers its task complete. The solution we used previously is [Stochastic Gradient Descent(SDG)](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/cac27683-d5f4-40b4-82ce-d708de8f5373/lessons/a4a80417-00cb-4a9c-8cc4-3a091414baa2/concepts/63798118390923) 


1-D to n-D Graphs 
--

1-D Graphs
--
In teh previous example we worked with 1-D graphs. The robot's motion and measurements were limited to one dimension i.e  the could only be performed either forwards or backwards. 

In such a case each constraint could be represented in the following form, 

1-D Measurement constraint: <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{(x_t-(x_{t-1}&plus;u_t))^2}{\sigma^2}" title="\frac{(x_t-(x_{t-1}+u_t))^2}{\sigma^2}" /></a>

1-D Motion constraint: <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{(z_t&space;-(x_{t-1}&plus;u_t))^2}{\sigma^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{(z_t&space;-(x_{t-1}&plus;u_t))^2}{\sigma^2}" title="\frac{(z_t -(x_{t-1}+u_t))^2}{\sigma^2}" /></a>

n-Dimensional Graphs 
---

In multi-dimensional systems, we must use matrices and covariances to represent the constraints. This generalization can be applied to system of 2-D, 3-D and really any n-number of dimensions. The equations for the constriants would look like so,

n-D Measurement constraint: <a href="https://www.codecogs.com/eqnedit.php?latex=(z_t-h(x_t,m_j))^{T}Q_{t}^{-1}(z_t-h(x_t,m_j))" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(z_t-h(x_t,m_j))^{T}Q_{t}^{-1}(z_t-h(x_t,m_j))" title="(z_t-h(x_t,m_j))^{T}Q_{t}^{-1}(z_t-h(x_t,m_j))" /></a>

n-D Motion constraint: <a href="https://www.codecogs.com/eqnedit.php?latex=(x_t-g(u_t,x_{t-1}))^{T}R_{t}^{-1}(x_t-g(u_t,x_{t-1}))" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_t-g(u_t,x_{t-1}))^{T}R_{t}^{-1}(x_t-g(u_t,x_{t-1}))" title="(x_t-g(u_t,x_{t-1}))^{T}R_{t}^{-1}(x_t-g(u_t,x_{t-1}))" /></a>

Where h() and g() represent the measurement and motion functions

Q_t and R_t are the covariances of the measurement and motion noise. These naming conventions are the same as the where in the [Localization](link) sections. 

The multi-dimensional formula for the sum of all constraints is presented below:

<a href="https://www.codecogs.com/eqnedit.php?latex=J_{GraphSLAM}=x_{0}^{T}\Omega&space;x_0&space;&plus;&space;\sum_t(x_t&space;-g(u_t,x_{t-1}))^{T}R_t^{-1}(x_t-g(u_t,&space;x_{t-1}))&plus;\sum_t(z_t-h(x_t,m_j))^{T}Q_t^{-1}(z_t-h(x_t,m_j))" target="_blank"><img src="https://latex.codecogs.com/gif.latex?J_{GraphSLAM}=x_{0}^{T}\Omega&space;x_0&space;&plus;&space;\sum_t(x_t&space;-g(u_t,x_{t-1}))^{T}R_t^{-1}(x_t-g(u_t,&space;x_{t-1}))&plus;\sum_t(z_t-h(x_t,m_j))^{T}Q_t^{-1}(z_t-h(x_t,m_j))" title="J_{GraphSLAM}=x_{0}^{T}\Omega x_0 + \sum_t(x_t -g(u_t,x_{t-1}))^{T}R_t^{-1}(x_t-g(u_t, x_{t-1}))+\sum_t(z_t-h(x_t,m_j))^{T}Q_t^{-1}(z_t-h(x_t,m_j))" /></a>


The first element is in the sum is the initial constrain. It sets the first robot pose to equal to the origin of the map. The covariance, <a href="https://www.codecogs.com/eqnedit.php?latex=\Omega_0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Omega_0" title="\Omega_0" /></a>, represents compplete confidence: 

<a href="https://www.codecogs.com/eqnedit.php?latex=\Omega_0=\begin{bmatrix}&space;\infty&space;&0&space;&0&space;\\&space;0&&space;\infty&space;&0&space;\\&space;0&&space;0&space;&&space;\infty&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Omega_0=\begin{bmatrix}&space;\infty&space;&0&space;&0&space;\\&space;0&&space;\infty&space;&0&space;\\&space;0&&space;0&space;&&space;\infty&space;\end{bmatrix}" title="\Omega_0=\begin{bmatrix} \infty &0 &0 \\ 0& \infty &0 \\ 0& 0 & \infty \end{bmatrix}" /></a>


Information Matrix and Vector 
--
Since we are working with mulit-dimensional graph and multi-dimensional constraints, we shift gears to a more intelligent data structure to work with our data.

The **Information Matrix** and **Information Vector** are two data structures that are used to store information from our constraints. 

Information Matrix - <a href="https://www.codecogs.com/eqnedit.php?latex=\Omega" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Omega" title="\Omega" /></a>

Information Vector - <a href="https://www.codecogs.com/eqnedit.php?latex=\xi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\xi" title="\xi" /></a>

The information matrix is the inverse of the co-variance matrix i.e higher certianity is represented with larger values in the information matrix; which is the opposite of the [covariance matrix](link) where complete certainity is represented by zero. 

The Matrix and vector hold  all of the poses and all of the features in the environment. Every off-diagonal cell in the matrix is a **link** between:

* Two poses
* A pose and a feature
* Two Features 

When no information is available about a link the cell has a value of zero. 

The information matrix and information vector exploit the additive property of the negative log likelihoods of constraints. For a system with linear measurement and motion models, the constriants can be populated into the information matrix and information vector in an additive manner. 

Below we see the start of a graph constructed by a robot as it moved around some environment. This graph contains 5 constraints:
[image013]

* 1 Initial Constraint
* 2 Motion Constraints 
* 2 Measurement Constraints

This graph will be used to demonstrate at a high level how the information matrix and vector can be populated. To begin with, they are populated successively with each constraint. 

The initial constraint will tie the pose x_0 to a value, usually zero. This constraint will populate one cell in the information matrix, and on in the information vector. 

Next, we examine the motion constriant between x_0 and x_1. 

A motion constraint will tie together two robot poses populating four cells in the matrix and two in the vector. 

Below are the cells that relate x_0 and x_1 to each other:
[image014]

Similarly, a measurment constraint will update four cells in the matrix and two in the vector. These are the cells that relate the feature to the pose from which it was measured, in this case m_1 to x_1. 

As state previously, the new formation supplied to the matrix and cector is additive. Continuing on....

The next constraint ties together poses x_1 and x_2 and the following constraint ties feature m_1 to the pose x_2. 

Once every constraint has been added to the information matrix and information vector, it is considered to be populated. It is common for the number of poses and features to be in the thousands or even tens of thousands. 

The information matrix is considered sparse because most off-diagonal elements are zero, since there is no relative information to tie them together. This sparsity is very valuable when it comes to solveing the systems of equations that is embedded into the information matrix and vector. 


Summary: 

* A motion constraint ties together two poses
* A measurement constraint ties together the feature and the pose from which is was measured
* Each operation updates 4 cells in the information matrix and 2 cells in the information vector
* All other cells remain 0. Matrix is called ‘sparse’ due to large number of zero elements
* Sparsity is a very helpful property for solving the system of equations


Inference 
--

Once the information matrix and information vector hve been populated, the path and map can be recovered by the following operation:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mu&space;=\Omega^{-1}\xi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu&space;=\Omega^{-1}\xi" title="\mu =\Omega^{-1}\xi" /></a>

The result is a vector (mue) defined over all the poses and features, containing the best estimate for each. This operation is very similar to what we encountered previously in the simple 1-D case. This time we add an additional structure. Same as before, all constrints are considered when computing the solution. 

Completing the above operation requires solving a system of equations. In small system, this is an easily realizable task, but as the size of the graph and matrix grows, computational efficieny becomes a big concern. 

The efficiency of this operation, specifically the matrix inversion, depends greatly on the **topology** of the system. 


Linear Graph
---

If the robot moves through the environment once without ever returning to a previously visited location, the topology is linear. Such a graph will produce a rather sparse matrix that with some effort, can be reordered to move all non-zero elements to near the diagonal. This will allow the above equation to be completed in linear time. 

Cyclical Graph
---

A more common topology is *cyclical* i.e the robot revisits a location that it has been to before, after some time has passed. In such a case, features in the environment will be linked to multiple poses, ones that are not consecutive, but spaced far apart. The further apart in time that these poses are, the more problematic. This is because such a martix cannot be reordered to move non-zero cells closer to the diagonal. The results is a matrix that is more computationally challengeing to recover. 

Variable Elimination
---

A variable elimination algorithm can be used to simplify the matrix, allowing for the inversion and product to be computer quicker. 

We apply variable elimination iteratively to remove all cyclical constraints. Variable elimination entails removing a variable (for example a feature) entirely from the graph and matrix. This can be done by adjusting existing links or ading new links to accommodate for the links that will be removed.

Recalling our spring analogy, variable elimination removes features, but keeps the *net* forces in the springs unaltered by adjusting the tension on other springs or adding new springs where needed. 

This process is demonstrated in the following two images:

**image 1** - The first image shows the grah, matrix, and vector as they where presented previously.

[image015]


**image 2** - The second image shows the elimation of m_1. In this process, the matrix will have five cells reset to zero (indicated in red), and four cells will have their values adjusted (indicated in green) to accommodate the variable elimination. Similarly, the information vector will have one cell removed and two adjusted.

[image016]

This process is repeated for all of the features, and in the end the matrix is defined over all robot poses. At this point, the same procedure as before can be applied <a href="https://www.codecogs.com/eqnedit.php?latex=\mu&space;=\Omega^{-1}\xi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu&space;=\Omega^{-1}\xi" title="\mu =\Omega^{-1}\xi" /></a>

Performing variable elimination on the information matrix/vector prior to performing inference is less computationally intense than attempting to solve the infernce problem directly on the unaltered data. 

In practice, the analytical inference method described above is seldom applied, because numerical methods are able to converge on a sufficiently accurate estimate in a fraction of the time. 

Nonlinear Constraints 
--
Before we talk about the numerical methods, which are better because they are able to converge on a sufficiently accurate estimate in a fraction of the time, we explore how nonlinear constraints are handled in GraphSLAM. 

When we talked about [Localization](link2) we were introducted to nonlinear motion and measurement models. The idea that a robot only moves in a linear fashion is very limiting. Therefore, it was mportant to understand how to work with nonlinear models. In localization, nonlinear models couldn't be applied directly, as they would have turned the Gaussian distribution into a much more complicated distribution that couldnt be computed in closed form (analytically, in a finite number of steps) 

This is also true of nonlinear models in SLAM- most motion and measurement constraints are nonlinear and must be linearized before they can be added to the information matrix and information vector. Otherwise, it would be impractical, if not impossible to solve the systems of equations analytically. 

Luckily, we will be able to apply the same procedure that we learned in the [EK](link3) lesson to lineaeize nonlinear constraints for SLAM. 

If you recall, a Taylor Series approximates a function using the sum of an infinite number of terms. A linear approximation can be computed by using only the first two terms and ignoring all higher order terms. In multi-dimensional models, the first derivative is replaced by a **Jacobian** 

*Jacobian* - A matrix of partial derivative.

Linearizing Constraints
---
A linearization of the measurement and motion constraints is the following 

<a href="https://www.codecogs.com/eqnedit.php?latex=g(u_t,x_{t-1})\simeq&space;g(u_t,&space;\mu_{t-1})&plus;&space;G_t(x_{t-1}-\mu_{t-1})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?g(u_t,x_{t-1})\simeq&space;g(u_t,&space;\mu_{t-1})&plus;&space;G_t(x_{t-1}-\mu_{t-1})" title="g(u_t,x_{t-1})\simeq g(u_t, \mu_{t-1})+ G_t(x_{t-1}-\mu_{t-1})" /></a>


<a href="https://www.codecogs.com/eqnedit.php?latex=h(y_t)\simeq&space;h(\mu_t)&plus;H_{j}^{i}(y_t-\mu_t)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?h(y_t)\simeq&space;h(\mu_t)&plus;H_{j}^{i}(y_t-\mu_t)" title="h(y_t)\simeq h(\mu_t)+H_{j}^{i}(y_t-\mu_t)" /></a>


To linearize each constraint, we need a value for <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{t-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{t-1}" title="\mu_{t-1}" /></a> or <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{t}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{t}" title="\mu_{t}" /></a> to linearize about. This value is important since the linearization of a nonlinear function can change significantly depending on which value we choose to do so about. 

A reasonable estimate for <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{t-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{t-1}" title="\mu_{t-1}" /></a> or <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{t}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{t}" title="\mu_{t}" /></a> :

When presented with a completed graph of nonlinear constraints, you can apply only the motion constraints to create a pose estimate, [x_0.....x_t]^T and use this primitive estimate in place of mue to linearize all of teh constraints. Then, onve all of the constraints are linearized and added to the matrix and vector, a solution can be computed as before, using <a href="https://www.codecogs.com/eqnedit.php?latex=\mu=\Omega^{-1}\xi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu=\Omega^{-1}\xi" title="\mu=\Omega^{-1}\xi" /></a>

This solution is unlikely to be an accurate solution. The pose vector used for linearization will be erroneous, since applying just the motion constraints will lead to a graph with a lof of drift, as errors accumulate with every motion. Errors in this initial pose vector will propagate through the calculations and affect the accuracy of the end result. This is especially so because the errors may increase in magnitude significantly during a poorly positioned linearization (where the estimated mu_t is far from reality, or the estimated mu_t lies on a curve where a small step in either direction will make a big difference. 

To reduce this error, we repeat the linearization process several times, each time using a better and better estimate to linearize the constraints about.

Iterative Optimization
---

The first iteration will see the constraints linearized about the pose estimate created using solely motion constraints. Then, the system of equations will be solved to produced a solution, mu. 

The next iteration will use this solution, mu as the estimate used to linearize about. The thought that this estimate would be a little better thant the previous, takes into account the measurement constraints too. 

This process continues, with all consequent iterations using the previous solution as the vector of poses to linearize the constraints about. Each solution incrementally improves on the previous, and after some number of iterations the solution converges. 

Summary
---
Nonlinear constraints can be linearized using Taylor Series, but this will most certainly introduce some error. To reduce this error, the linearization of every constraint must occur as close as possible to the true location of the pose or measurment relating to the constraint. To accomplish this, an iteative solution is used, wheree the point of linearization is improved with every iteration. After several iterations, the results, mu, becomes a much more reasonable estimate for teh true locations of all robot poses and features. 

The workflow for GraphSLAM is summarized as follows:

* Collect data, create graph of constraints,
* Until convergence:
  * Linearize all constraints about an estimate, μ and add linearized constraints to the information matrix & vector,
Solve system of equations using 
μ = Ω^(−1)*ξ 



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
