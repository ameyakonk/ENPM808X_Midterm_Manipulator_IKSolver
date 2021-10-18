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
 * @file Forward_kinematics.hpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This header file contains the Forward Kinematics class members and attributes
 *        Class to call solve_FK,getter and setter methods
 *
 */

// Header Files
#ifndef INCLUDE_FORWARD_KINEMATICS_HPP_
#define INCLUDE_FORWARD_KINEMATICS_HPP_
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "Eigen/Core"
#define PI 3.14

/**
 * @class Forward_Kinematics
 * @brief The following Class contains all the methods,attributes of Forward Kinematics Class.
 *        It provide methods to solve the forward kinematics of a robotic manipulator.
 *
 *
 */
class Forward_Kinematics {
 private:
  std::vector<double> input_joint_angles;  // robot manipulator joint angles
  // output joint coordinates of all the joint
  std::vector<double> output_joint_coordinates;
  std::vector<double> output_joint_angles;  // output joint angles of the arm
  std::vector<double> current_robot_pose;  // the current robot pose

 public:
  /**
   * @fn void solve_FK(std::vector<double>)
   * @brief this function will calculate the end effector position
   rom the given input_joint_angles.
   *
   * @param input_joint_angles these are the input joint angles of the robotic manipulator
   */

  void solve_FK(std::vector<double> input_joint_angles);
  /**
   * @fn void set_output_coordinates(std::vector<double>)
   * @brief It sets the output_coordinates(input) to the output_joint_coordinates
   *
   * @param _output_joint_coordinates
   * @return None
   */
  void set_output_coordinates(std::vector<double> _output_joint_coordinates);
  /**
   * @fn void set_output_angles(std::vector<double>)
   * @brief It sets the given input to output_joint_coordinates
   *
   * @param _output_joint_angles
   * @return None
   */
  void set_output_angles(std::vector<double> _output_joint_angles);
  /**
   * @fn void set_input_angles(std::vector<double>)
   * @brief It sets the given input to input_joint_angles.
   *
   * @param _input_joint_angles
   * @return None
   */
  void set_input_angles(std::vector<double> _input_joint_angles);
  /**
   * @fn void set_current_pose(std::vector<double>)
   * @brief It sets the given input to current_robot_pose
   *
   * @param _current_robot_pose
   * @return None
   */
  void set_current_pose(std::vector<double> _current_robot_pose);
  /**
   * @fn std::vector<double> get_output_coordinates()
   * @brief Getter method for returning output_joint_coordinates
   *
   * @return output_joint_coordinates
   */
  std::vector<double> get_output_coordinates();
  /**
   * @fn std::vector<double> get_output_angles()
   * @brief Getter Method for returning output_joint_angles
   *
   * @return output_joint_angles
   */
  std::vector<double> get_output_angles();
  /**
   * @fn std::vector<double> get_current_pose()
   * @brief Getter method for returning the current_robot_pose
   *
   * @return current_robot_pose
   */
  std::vector<double> get_current_pose();
  /**
   * @fn std::vector<double> get_input_angles()
   * @brief Getter method for getting the input_joint_angles
   *
   * @return input_joint_angles
   */
  std::vector<double> get_input_angles();
};
#endif  // INCLUDE_FORWARD_KINEMATICS_HPP_
