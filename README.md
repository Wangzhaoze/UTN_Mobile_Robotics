# UTN_Mobile_Robotics.md


![](https://img.shields.io/github/stars/pandao/editor.md.svg) 
![](https://img.shields.io/github/forks/pandao/editor.md.svg) 
![](https://img.shields.io/github/tag/pandao/editor.md.svg)
![](https://img.shields.io/github/release/pandao/editor.md.svg)
![](https://img.shields.io/github/issues/pandao/editor.md.svg)
![](https://img.shields.io/bower/v/editor.md.svg)

## 1. Course Introduction

![](https://www.utn.de/files/2022/08/Mobile-robotics-1-c-Unsplash-860x576.jpg)

For more details please follow this link: [Page of Course](https://www.utn.de/en/2022/09/13/mobile-robotics/)

This is an online course of topic mobile robots of University of Technology Nuremberg which strated in November 2022. The course mainly introduces:
- Kinematics model of mobile robots and Lidar
- Bayesian probability model
- Localization based on Lidar and wheel odometry with the help of [Particle Filtering](https://en.wikipedia.org/wiki/Particle_filter#:~:text=Particle%20filters%2C%20or%20sequential%20Monte,processing%20and%20Bayesian%20statistical%20inference.)
- [Fast_SLAM](http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf).

All these knowledge are simulated and verified on a robot simulation platform [Webots](https://cyberbotics.com/), which is of great help to understand the various formulas and concepts in the Simultaneous Localization And Mapping (SLAM) of mobile robots. Thanks to Prof.Wolfram Burgard and other teaching assistants, the reference solutions of exercises and slides are provided by them.


## 2. Webots Simulator
If you have not yet installed the [Webots Simulator](https://cyberbotics.com/), please do so. We recommend the latest version, 2022b, however, you can also use version 2022a if that works better on your system. Then, follow the [official tutorial](https://cyberbotics.com/doc/guide/tutorials) to learn the basics of the simulator. 



## 3. Content

#### 3.1 Locomotion and Lidar

- In this assignment, you will use the Webots simulator to control a Pioneer 3-DX robot. First, you are asked to implement a differential drive controller using the formulas you learned in this module. Second, you have to use the robot’s LiDAR to find the closest surface and navigate to it. Finally, you will devise and implement a wall-following algorithm. Before starting with the actual assignment, you should read and follow the instructions in the Preliminaries section in order to familiarize yourself with the simulator.
  
<video controls>
  <source src="Modul1_Locomotion_and_Lidar_Sensors/res/ex1_task1.mp4" type="video/mp4">
</video>

#### Bayes Filter

#### Motion and Sensor Model

#### Particle Filter

#### SLAM

![](https://github.com/Wangzhaoze/UTN_Mobile_Robotics/blob/9267130cca50bb51588420b8b1cf7c3ead06183d/Modul4_Particle_Filter/res/particle_filter.png)
