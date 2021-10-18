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
 * @file Forward_kinematics.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This file contains the Forward Kinematics methods used to find
 *        out the end-effector coordinates of the robotic manipulator.
 *
 *
 */

// Header Files.
#include "Forward_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"
#include "Inverse_kinematics.hpp"
/**
 * @fn void solve_FK(std::vector<double>)
 * @brief This Function will compute the ForwardKinematics
 *        for the given input joint angles and
 *        sets the end effector coordinates to output_coordinates.
 *
 * @param _input_joint_angles is the Robot Manipulator Joint angles
 * @return None
 */
void Forward_Kinematics::solve_FK(
    const std::vector<double> &_input_joint_angles) {
  Inverse_Kinematics I;  // Calling an object from the Inverse Kinematics Class
  // set the dh_d for the given inputs
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  // set the df_a for the given inputs
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  // set the dh_alpha for the given inputs.
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  // Creating a 4 X 4 matrix for the transformation matrix
  Eigen::MatrixXd trans_mat;
  // Created a 4 X 4 matrix for the final transformation matrix
  Eigen::Matrix<double, 4, 4> final_transformation_matrix;
  // Have issues with accessing dh_d from the IK class.
  std::vector<double>::size_type i = 0;
  // for (i = 0; i < 6; i++) {
  //  cout << I.get_dh_d()[i] << endl;
  // }
  // Iterating through each transformation matrix and
  /* calculating the final_transformation_matrix
   * by multiplying individual trans_mat */
  trans_mat.resize(4, 4);
  for (int r = 0; r < 6; r++) {
    trans_mat << cos(_input_joint_angles[i]), (-cos(I.get_dh_alpha()[i]))
        * sin(_input_joint_angles[i]), sin(I.get_dh_alpha()[i])
        * sin(_input_joint_angles[i]), I.get_dh_a()[i]
        * cos(_input_joint_angles[i]), sin(_input_joint_angles[i]), cos(
        I.get_dh_alpha()[i]) * cos(_input_joint_angles[i]), (-sin(
        I.get_dh_alpha()[i]) * cos(_input_joint_angles[i]), I.get_dh_a()[i]
        * sin(_input_joint_angles[i])), 0, sin(I.get_dh_alpha()[i]), cos(
        I.get_dh_alpha()[i]), I.get_dh_d()[i], 0, 0, 0, 1;
    final_transformation_matrix *= trans_mat;
    i++;
  }
  /* extracting X,Y,Z from the final_transformation_matrix. */
  // to store the end-effector(X,Y,Z) positions
  std::vector<double> end_effector_coordinates;
  end_effector_coordinates.push_back(final_transformation_matrix(1, 4));
  end_effector_coordinates.push_back(final_transformation_matrix(2, 4));
  end_effector_coordinates.push_back(final_transformation_matrix(3, 4));
  // setting the output_coordinates as end_effector_coordinates
  set_output_coordinates(end_effector_coordinates);
}
/**
 * @fn void set_output_coordinates(std::vector<double>)
 * @brief It sets the output_coordinates(input) to the output_joint_coordinates
 *
 * @param _output_joint_coordinates
 * @return None
 */
void Forward_Kinematics::set_output_coordinates(
    const std::vector<double> &_output_joint_coordinates) {
  output_joint_coordinates = _output_joint_coordinates;
}
/**
 * @fn void set_output_angles(std::vector<double>)
 * @brief It sets the given input to output_joint_coordinates
 *
 * @param _output_joint_angles
 * @return None
 */
void Forward_Kinematics::set_output_angles(
    const std::vector<double> &_output_joint_angles) {
  output_joint_angles = _output_joint_angles;
}
/**
 * @fn void set_input_angles(std::vector<double>)
 * @brief It sets the given input to input_joint_angles.
 *
 * @param _input_joint_angles
 * @return None
 */
void Forward_Kinematics::set_input_angles(
    const std::vector<double> &_input_joint_angles) {
  input_joint_angles = _input_joint_angles;
}
/**
 * @fn void set_current_pose(std::vector<double>)
 * @brief It sets the given input to current_robot_pose
 *
 * @param _current_robot_pose
 * @return None
 */
void Forward_Kinematics::set_current_pose(
    const std::vector<double> &_current_robot_pose) {
  current_robot_pose = _current_robot_pose;
}
/**
 * @fn std::vector<double> get_output_coordinates()
 * @brief Getter method for returning output_joint_coordinates
 *
 * @return output_joint_coordinates
 */
std::vector<double> Forward_Kinematics::get_output_coordinates() {
  return output_joint_coordinates;
}
/**
 * @fn std::vector<double> get_output_angles()
 * @brief Getter Method for returning output_joint_angles
 *
 * @return output_joint_angles
 */
std::vector<double> Forward_Kinematics::get_output_angles() {
  return output_joint_angles;
}
/**
 * @fn std::vector<double> get_current_pose()
 * @brief Getter method for returning the current_robot_pose
 *
 * @return current_robot_pose
 */
std::vector<double> Forward_Kinematics::get_current_pose() {
  return current_robot_pose;
}
/**
 * @fn std::vector<double> get_input_angles()
 * @brief Getter method for getting the input_joint_angles
 *
 * @return input_joint_angles
 */
std::vector<double> Forward_Kinematics::get_input_angles() {
  return input_joint_angles;
}
