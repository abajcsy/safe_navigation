# Provably Safe Autonomous Navigation in Unknown Environments

This code implements an [efficient reachability-based framework for provably safe autonomous navigation in unknown environments](https://arxiv.org/abs/1905.00532) in simulation and for a Turtlebot 2 robot in hardware. 

## Requirements
To run the basics of the repository, you need the following:
* MATLAB R2017b
* [helperOC](https://github.com/HJReachability/helperOC) -- Optimal control toolbox for HJ reachability analysis. It has all the essential helper functions for reachability computations, dynamical system definitions, etc. 
* [toolboxls](https://bitbucket.org/ian_mitchell/toolboxls/src/default/) -- Level Set Methods Toolbox by Ian Mitchell

While not essential, to run the fancier planners or to use fancier environments you may need one or all of the following:
* [WayPtNav](https://vtolani95.github.io/WayPtNav/) for the vision-based neural network planner
* [Stanford Building Dataset](http://buildingparser.stanford.edu/dataset.html) for running simulated experiments in the stanford buildings
* [RTAB-Map](http://introlab.github.io/rtabmap/) for running real-time SLAM onboard the Turtlebot. 

### Notes
* In this version, the `master` branch is currently very out of date. Use the `experimental/planning` for the most functionality.

## Functionality
The current repository supports and/or implements a variety of dynamics models, sensors, planners, environment representations, and safety computation methods. 

#### Dynamical system models
* 3D Dubins car model with states (x,y,theta) as x-y position and heading. Control input is turning rate.
* 4D unicycle model with states (x,y,theta,v) as x-y position, heading, and velocity. Control inputs are turning rate and acceleration. 

#### Sensor types
* Monocular front-facing camera
* LiDAR

#### Planners
* Rapdly-exploring random trees ([RRTs](http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf))
* [Spline-based planner](http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf)
* Vision-based neural-network planner ([WayPtNav](https://arxiv.org/pdf/1903.02531.pdf))

#### Environments
* Hand-specified set of square or circle obstacles
* Rooms loaded from the [Stanford Building Dataset](http://buildingparser.stanford.edu/dataset.html)
* Occupancy maps produced by [SLAM](http://introlab.github.io/rtabmap/)

#### Safety Computations
* Hamilton Jacobi-Isaacs Variational Inequality (HJI-VI), as implemented by the Level Set Toolbox
* Warm-started HJI-VI, as described in Section B1 [here](https://arxiv.org/pdf/1905.00532.pdf)
* Local-update HJI-VI, as described in Section B2 [here](https://arxiv.org/pdf/1905.00532.pdf)

## Setting up an experiment
Every simulation or hardware experiment begins with an experimental parameters file that defines what dyanmical system model you would like to use, what sensor is onboard the robot, if you are in simulation or in hardware, and what kind of safety update you would like to perform. 

All the parameter files can be found under the `src/experiments` folder. They are categorized by dynamical system model dimension and sensor type. As an example, look at the file named `src/experiments/3D/camera/car3DHJICameraRRT.m` for all the types of parameters that you can setup and play with.

## Running In Simulation
(to be documented)

## Running In Hardware
(to be documented)
