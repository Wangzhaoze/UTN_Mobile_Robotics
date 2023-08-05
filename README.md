# UTN_Mobile_Robotics.md


![](https://img.shields.io/github/stars/pandao/editor.md.svg) 
![](https://img.shields.io/github/forks/pandao/editor.md.svg) 
![](https://img.shields.io/github/tag/pandao/editor.md.svg)
![](https://img.shields.io/github/release/pandao/editor.md.svg)
![](https://img.shields.io/github/issues/pandao/editor.md.svg)
![](https://img.shields.io/bower/v/editor.md.svg)

## 1. Course Introduction

![](https://www.utn.de/files/2022/08/Mobile-robotics-1-c-Unsplash-860x576.jpg)

> For more details please follow this link: [Page of Course](https://www.utn.de/en/2022/09/13/mobile-robotics/)

This is an online course of topic mobile robots of University of Technology Nuremberg which strated in November 2022. The course mainly introduces:
- Kinematics model of mobile robots and Lidar
- Bayesian probability model
- Localization based on Lidar and wheel odometry with the help of [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter#:~:text=Particle%20filters%2C%20or%20sequential%20Monte,processing%20and%20Bayesian%20statistical%20inference.)
- [Fast_SLAM](http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf).

All these knowledge are simulated and verified on a robot simulation platform [Webots](https://cyberbotics.com/), which is of great help to understand the various formulas and concepts in the Simultaneous Localization And Mapping (SLAM) of mobile robots. Thanks to Prof.Wolfram Burgard and other teaching assistants, the reference solutions of exercises and slides are provided by them.


## 2. Webots Simulator
If you have not yet installed the [Webots Simulator](https://cyberbotics.com/), please do so. We recommend the latest version, 2022b, however, you can also use version 2022a if that works better on your system. Then, follow the [official tutorial](https://cyberbotics.com/doc/guide/tutorials) to learn the basics of the simulator. 



## 3. Content

#### 3.1  Locomotion and Lidar


- In this assignment, you will use the Webots simulator to control a Pioneer 3-DX robot. First, you are asked to implement a differential drive controller using the formulas you learned in this module. Second, you have to use the robot’s LiDAR to find the closest surface and navigate to it. Finally, you will devise and implement a wall-following algorithm. Before starting with the actual assignment, you should read and follow the instructions in the Preliminaries section in order to familiarize yourself with the simulator.

<br>

- Differential Drive of Robots
![](https://github.com/Wangzhaoze/UTN_Mobile_Robotics/blob/c2f415d056f7499e5f53ee6dade7150588d3c9d4/Modul1_Locomotion_and_Lidar_Sensors/Lecture/differential_drive.png)

<br>


- Simulation/video
  
<video id="video" controls="" preload="none" poster="http://media.w3.org/2010/05/sintel/poster.png">
  <source id="mp4" src="https://www.youtube.com/watch?v=33KYJCG2_0c" type="video/mp4">
  <p>Your user agent does not support the HTML5 Video element.</p>
</video>




#### 3.2  Bayes Filter

- Consider a household robot equipped with a camera. It operates in an apartment with two rooms: a living room and a bedroom. The robot runs an artificial neural network that can recognize a living room in the camera image. Further, the robot can perform a switch-room action, i.e., it moves to the living room if it is in the bedroom, and vice versa. Neither the recognition nor the motion controller is perfect. 
    
- From previous experience, you know that the robot succeeds in moving from the living room to the bedroom with a probability of 0.7, and with a probability of 0.8 in the other direction. The probability that the neural network indicates that the robot is in the living room although it is in the bedroom is given, and the probability that the network correctly detects the living room is given.

- Unfortunately, you have no knowledge about the current location of the robot. However, after performing the switch-room action, the neural network indicates that the robot is not in the living room. After performing the switch-room action for the second time, the network again indicates not seeing a living room. Use the Bayes filter algorithm to compute the probability that the robot is in the bedroom after performing the two actions.

<br>

- Markov Localization
![](https://www.researchgate.net/profile/Sebastian-Hoeffner-2/publication/272490415/figure/fig1/AS:645701503553546@1530958643508/Markov-localization-A-one-dimensional-corridor-with-indistinguishable-doors-From.png)

<br>



#### 3.3  Motion & Sensor Model

The last module introduced the recursive Bayes filter. To implement it in practice, one requires the transition model p( xt | xt-1, ut） and the measurement or sensor model P (z | x). In the previous assignment, the necessary values of these functions were provided. In real-world applications, however, the transition and sensor model have a more complex form and need to fit the robot hardware at hand. In this module, you will learn a transition model that approximates the uncertainty in the motion of a wheeled robot, the odometry-based motion model. You will further learn the beam-based sensor model for range sensors.

<br>

To Do List:
-	model the motion uncertainty of a wheeled robot
-	implement an algorithm that samples from the odometry-based motion model
-	understand and apply a sensor model to estimate the likelihood of a range measurement

<br>

- Motion Model
![](https://github.com/Wangzhaoze/UTN_Mobile_Robotics/blob/9267130cca50bb51588420b8b1cf7c3ead06183d/Modul3_Motion_and_Sensor_Model/res/odometry%20motion%20model.png)

<br>

- Sensor Model
![](https://github.com/Wangzhaoze/UTN_Mobile_Robotics/blob/9267130cca50bb51588420b8b1cf7c3ead06183d/Modul3_Motion_and_Sensor_Model/res/sensor%20model.png)

<br>


#### 3.4  Particle Filter
- The recursive Bayes filter can be evaluated analytically under certain circumstances. For example, if the state space is discrete with a relatively low number of states or the motion model, the sensor model, and the initial 
belief all follow a Gaussian distribution. However, in the case of a continuous state space and a belief that follows a general probability distribution, we have to rely on some approximation of the belief. Consider the 
scenario of localizing a robot using identical landmarks: In case the robot observes only a single landmark, the resulting belief of the pose would be a multi-modal distribution, with one peak near each landmark. It is not 
trivial to represent such a distribution by a parametric function.

<br>


- Non-parametric Bayes filters can approximate arbitrary belief distributions. In this module, you will learn two non-parametric filters: the discrete Bayes filter and the particle filter, whereby the focus will be on the particle filter. 

<br>


- Particle Filter
![](https://github.com/Wangzhaoze/UTN_Mobile_Robotics/blob/9267130cca50bb51588420b8b1cf7c3ead06183d/Modul4_Particle_Filter/res/particle_filter.png)



#### 3.5  Fast_SLAM

- In the previous module, you learned how to localize a robot using a particle filter. There, the algorithm was provided with a map that was constructed from the ground truth data obtained from 
the simulator. In practice, a map can be recovered from sensor data if the robot trajectory is known, a.k.a. mapping with known poses. The problem of Simultaneous Localisation and 
Mapping (SLAM) is more complex. In SLAM, the robot needs to estimate its pose while mapping the environment at the same time. That is often described as a chicken-or-egg problem since 
localization requires a map and robot poses are needed for map creation.

<br>

- In this module, you will first obtain an introduction to the SLAM problem. Next, you will learn about the Kalman Filter, a widely used state estimation method for Gaussian processes. Finally, you will 
learn the FastSLAM algorithm, which is a SLAM solution based on the particle filter and Kalman Filter approaches. 

<br>




