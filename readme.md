# ENPM808X_Midterm_RRTStar
[![Build Status](https://app.travis-ci.com/ameyakonk/ENPM808X_Midterm_RRTStar.svg?branch=master)](https://app.travis-ci.com/ameyakonk/ENPM808X_Midterm_RRTStar)
[![Coverage Status](https://coveralls.io/repos/github/ameyakonk/ENPM808X_Midterm_RRTStar/badge.svg?branch=master)](https://coveralls.io/github/ameyakonk/ENPM808X_Midterm_RRTStar?branch=master)
---
## Overview

A robotic manipulator is used to direct material without direct physical contact with the operator. It was developed to handle hazardous materials or access places that are not easily accessible by humans. At Acme, the manipulator will be used for picking heavy containers from the conveyer belt of the production line and placing it on the packaging line. The manipulator will have four degrees of freedom and will be placed on a support structure, between the production and packaging line conveyor belts.

## Design and development process

The software design includes five blocks namely kinematics, path planning, control system, simulation, and testing. Incremental Rotary encoders will be used at each motor link to calculate the angle moved by a particular arm.

1. Kinematics 
    This block consists of two sections, Inverse Kinematics and Forward Kinematics.
    
2. Path planning
    The path planning block will deduce an optimal path based on the start and target point coordinates, mechanical constraints such as link lengths, and angular     constraints. The algorithm used for path planning is RRT*. A brief introduction to RRT* is given below.
    2.1 RRT* 
      The algorithm records the distance each vertex has traveled relative to its parent vertex. This is referred to as the vertex. After the closest node is           found in the graph, a neighborhood of vertices in a fixed radius from the new node is examined. If a node with a cheaper cost than the proximal node is           found, the cheaper node replaces the proximal node. After a vertex has been connected to the cheapest neighbor, the neighbors are again examined. Neighbors       are checked if being rewired to the newly added vertex will make their cost decrease. If the cost does indeed decrease, the neighbor is rewired to the newly       added vertex. 

3. Control system
    Based on the feedback from the Kinematics block of the current position of the end effector and the path defined by the Path planning block, the Control           system block ensures that the path is followed by the links and the end-effector reaches the desired target point within a pre-defined error margin.

4. Simulation
    Matlab will be used for the simulation.

6. Testing
    Software testing will be done to check the repeatability, effectiveness of the output.
    Unit tests will be performed to check the values of encoders and the working of specific functions.


file:///home/ameya/Downloads/WhatsApp%20Image%202021-10-06%20at%203.33.33%20PM.jpeg![image](https://user-images.githubusercontent.com/78075049/136273325-7f6e57c6-8a1d-4dbf-bfb1-7363d3868c99.png)

## Team Members

Sprint 1 : Driver- Ameya Konkar ; Navigator- Rahul Karanam

Sprint 2 : Driver- Rahul Karanam ; Navigator- Ameya Konkar

## Standard install via command-line
```
git clone --recursive https://github.com/ameyakonk/ENPM808X_Midterm_RRTStar.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```
## Product Backlog

https://docs.google.com/spreadsheets/d/1gIb1oDr5XimfNZunjdkT8HPoZb-tI-Xpfw4TqqyA0Ks/edit?usp=sharing




