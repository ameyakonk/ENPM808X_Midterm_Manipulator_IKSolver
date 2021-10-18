# ENPM808X_Midterm_Manipulator_IK_Solver
[![Build Status](https://app.travis-ci.com/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver.svg?branch=Phase1)](https://app.travis-ci.com/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver)
[![Coverage Status](https://coveralls.io/repos/github/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver/badge.svg?branch=Phase1)](https://coveralls.io/github/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver?branch=Phase1)

## Project Description 

## Overview
This repository will be used for Solving Inverse Kinematics for the robotic arm at ACME Robotics.
It is currently used for only manipulators having six degree of freedom.
The input coordinates [X,Y,Z] are send as an input to the IK solver to get the output_joint_angles.
We have created the FK solver in order to check for robot arm constraints are in the bias range or not.
We use FK_solver in order to check for the output_bias and to cancel out singularities of the robot arm.

This software will compute the trajectory  when you input your desired location which will return a vector of all the joint angles for the robot.
We simulate our tracjectory using matplotlib for all the output coordinates covered till the desired location.

This IK solver can be integrated with any six degree of robot manipulator.
A robotic manipulator is used to direct material without direct physical contact with the operator. It was developed to handle hazardous materials or access places that are not easily accessible by humans. At Acme, the manipulator will be used for picking heavy containers from the conveyer belt of the production line and placing it on the packaging line. The manipulator will have four degrees of freedom and will be placed on a support structure, between the production and packaging line conveyor belts.

## Background

We are considering Stanford Arm for our IK solver algorithm.

The main applications of this software is to solve the inverse kinematics of the robot manipulator given the end effector coordinates.

We are developing an inverse kinematics solver for 6 DOF serial Manipulators at ACME robotics.
- When equipped with a 3D Camera it can be used as a Vision-Guided Inspection Robot.

- The 3D camera is being used for identification and detection of an object in an assembly line.

- The output pose from the camera is given as an input to the Robot which will calculate the joint angles using IK solver and place the object in the desired location set by the user.

-This robot can be  used for random part inspection in an assembly line.



## Features

- UML diagrams representing the contracts.

- C++11 and C++14 and implemented the code using best software practices.

- Continous Integration (Travis Badges) for reporting the build phase over changes.

- It can be integrated with a vision to give a real time vision guidance process in a manufacturing process.

- It can be used for various applications such as Error Proofing,Palletizing and Depalletizing,Real time guidance and bin picking.

- These manipulators can be used for multiple parts.




## Approach



We have created two solvers methods , FK solver which will calculate the forward kinematics of the robot given the input_joint_angles.

We Use Denavit-Hartenberg representation for solving Inverse Kinematics.

IK Solver which will calculate the joint angles based upon the end effector coordinates.

For the simulation we take all the joint angle coordinates returned by the IK solver are visualized through the matplotlib.

Created a test suite to check for all the methods.

We followed Test Driven Approach - Pair Programmning and completed the process using Agile Iterative Process.

## Design and development process

The software design includes three blocks namely kinematics, simulation, and testing. Incremental Rotary encoders will be used at each motor link to calculate the angle moved by a particular arm.

1. Kinematics 
    This block consists of two sections, Inverse Kinematics and Forward Kinematics.
    
2.Simulation
    Matplotlib will be used for the simulation.

3.Testing
    Software testing will be done to check the repeatability, effectiveness of the output.
    
    Unit tests will be performed to check the values of encoders and the working of specific functions.
    
    We have created unit tests to check the output_bias as to adhere to the safety constraints of the robot manipulator.
    
---

## Personnel

### Rahul Karanam

UID:118172507

Master's Student at University of Maryland,College Park

### Ameya Konkar 

UID:118191058


Master's Student at University of Maryland,College Park

## Travis and Coveralls

Badges for Travis and Coveralls are located at the top of the readme file. 
Additional information on building the software to test for coverage is shown below. 

```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
Running the above code will generate a index.html page.
Check the build/coverage folder and view it in a web browser.
## License

We are going with a BSD 3-Clause License.


## Links to Agile Iterative Process (AIP) 
---
Please find the below google docs for the Product backlog and sprint planning sheets.

To Project Backlog (Product Backlog, Iteration Backlogs, and Work Log):
https://docs.google.com/spreadsheets/d/1uzkHEVrKEj08UWGMlCNrI8JZQGKhPk-L/edit#gid=1646452053


To Sprint Planning Notes/Review:

https://docs.google.com/document/d/1zEVBw1UfMGHF_3bLddfBvVKr5AerR_yLoXSyQXR5pro/edit?usp=sharing

---

## Dependencies 

We have two dependancies for the software to function properly.

- Eigen :  We use this package for all our kiematics calculations.
- Matplotlib : It is used for simulating our Robot IK solver method.



## Known Issues/Bugs

---
While running the main source code , we are getting segementation core dumped due to issues with the eigen package.
Need to resolve in the next iteration.

Having issue while installing matplotlib.

Added to the backlog for the next iteration.

The issue where the source file might have issue is when the dependancies are not installed properly.

Make sure you install the correct packages.

Check out this link for installing Eigen:
https://eigen.tuxfamily.org/dox/GettingStarted.html

Check out this link for installing matplotlib:
https://matplotlib-cpp.readthedocs.io/en/latest/

---

### Building the Program and Tests

```
git clone --recursive https://github.com/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run program: ./app/shell-app
```
### To Run the Test

```
Run tests: ./test/cpp-test

```
### Generating Doxygen Docs

``` 
sudo apt-install doxywizard
run doxywizard
source it to ENPM808X_Midterm_Manipulator_IKSolver/
run the doxygen
```
check your destination folder for the doxygen docs.
--> Check doxygen folder for the doxygen documentation
    
---


## License

---
BSD 3-Clause License.


Copyright (c) 2021, ACME Robotics, Rahul Karanam , Ameya Konkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

---




