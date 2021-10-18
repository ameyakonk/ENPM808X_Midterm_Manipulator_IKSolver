/**
 * BSD 3-Clause License
 * Copyright (c) 2021, ACME Robotics, Rahul Karanam , Ameya Konkar
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file Inverse_kinematics.hpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This header file contains the Forward Kinematics class members and attributes
 *        Class to call solve_FK,getter and setter methods
 *
 */

#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include "Eigen/Core"
#define PI 3.14

/**
 * @class Inverse_Kinematics
 * @brief The following Class contains all the methods,attributes of Inverse Kinematics Class.
 *        It provide methods to solve the inverse kinematics of a robotic manipulator.
 *
 *
 */
class Inverse_Kinematics {
   private:
	   std::vector <double> input_joint_angles; 				// robot manipulator input joint angles
	   std::vector <double> input_joint_coordinates;	   // robot manipulator input joint coordinates
	   std::vector <double> output_joint_coordinates;		// robot manipulator output joint angles
	   std::vector <double> output_joint_angles;				// robot manipulator output joint angles
	   std::vector <double> current_robot_pose;				// robot manipulator current position
	   std::vector <double> link_lengths;						// robot manipulator link angles
	   std::vector <double> joint_angle_constraints;		// robot manipulator joint angle constraints
	   std::vector <double> dh_a;									// robot manipulator dh parameter-a
	   std::vector <double> dh_d;									// robot manipulator dh parameter-d
	   std::vector <double> dh_alpha;							// robot manipulator dh parameter-alpha
	   std::vector <double> output_bias;						// robot manipulator output bias
   public:
	   void solve_IK(std::vector<double> , std::vector<double>);  	// method to compute Inverse kinematics
	   void set_input_coordinates(std::vector<double>);				// method to set input joint coordinates 
	   void set_output_coordinates(std::vector<double>); 				// method to set output joint coordinates
	   void set_output_angles(std::vector<double>);  					// method to set input joint angles
	   void set_input_angles(std::vector<double>);						// method to set current manipulator end effector position
	   void set_current_pose(std::vector<double>);						
	   void set_dh_a(std::vector<double>);									// method to set dh parameter-a
	   void set_dh_d(std::vector<double>);									// method to set dh parameter-d
	   void set_dh_alpha(std::vector<double>);							// method to set dh parameter-d 
	   std::vector<double> get_input_coordinates();						// method to get input joint coordinates
	   std::vector<double> get_output_coordinates();					// method to get output joint coordinates
      std::vector<double> get_input_angles();							// method to get input joint angles
      std::vector<double> get_output_angles();							// method to get output joint angles
      std::vector<double> get_current_pose();							// method to get current end effector position
	   std::vector<double> get_dh_a();										// method to get dh parameter-a
      std::vector<double> get_dh_d();										// method to get dh parameter-d
	   std::vector<double> get_dh_alpha();									// method to get dh parameter-a
	   void reset_pose();														// method to reset the manipulator
	   std::vector<double> convert_input_angles_to_rotation_matrix(std::vector<double>); // method to convert pitch, yaw, roll to rotation matrix
};
#endif
