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
#include "Eigen/Dense"
#include "Inverse_kinematics.hpp"
using Eigen::Matrix;
//using namespace Eigen;
/**
 * @fn void solve_FK(std::vector<double>)
 * @brief This Function will compute the ForwardKinematics
 *        for the given input joint angles and
 *        sets the end effector coordinates to output_coordinates
 *        end_effector pose to current_pose.
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
  typedef Matrix<double, 4, 4> Matrix4f;

  // Created a 4 X 4 matrix for the final transformation matrix
  Matrix4f final_transformation_matrix;
  // Updating the values of alpha,d,a and the theta in the final transformation matrix.
  final_transformation_matrix(0, 0) = cos(_input_joint_angles[0])
      * (cos(_input_joint_angles[1])
          * (cos(_input_joint_angles[5]) * cos(_input_joint_angles[3])
              * cos(_input_joint_angles[4])
              - sin(_input_joint_angles[3]) * sin(_input_joint_angles[5]))
          - sin(_input_joint_angles[4]) * sin(_input_joint_angles[1])
              * cos(_input_joint_angles[5]))
      - sin(_input_joint_angles[0])
          * (cos(_input_joint_angles[5]) * cos(_input_joint_angles[4])
              * sin(_input_joint_angles[3])
              + cos(_input_joint_angles[3]) * sin(_input_joint_angles[5]));
  final_transformation_matrix(0, 1) = sin(_input_joint_angles[0])
      * (cos(_input_joint_angles[1])
          * (cos(_input_joint_angles[5]) * cos(_input_joint_angles[3])
              * cos(_input_joint_angles[4])
              - sin(_input_joint_angles[3]) * sin(_input_joint_angles[5]))
          - sin(_input_joint_angles[4]) * sin(_input_joint_angles[1])
              * cos(_input_joint_angles[5]))
      + cos(_input_joint_angles[0])
          * (cos(_input_joint_angles[5]) * cos(_input_joint_angles[4])
              * sin(_input_joint_angles[3])
              + cos(_input_joint_angles[3]) * sin(_input_joint_angles[5]));
  final_transformation_matrix(0, 2) = -sin(_input_joint_angles[1])
      * cos(_input_joint_angles[5]) * cos(_input_joint_angles[3])
      * cos(_input_joint_angles[4])
      - sin(_input_joint_angles[5]) * cos(_input_joint_angles[1])
          * cos(_input_joint_angles[5])
      + sin(_input_joint_angles[5]) * sin(_input_joint_angles[1])
          * sin(_input_joint_angles[3]);
  final_transformation_matrix(0, 3) = 0;
  final_transformation_matrix(1, 0) = cos(_input_joint_angles[0])
      * (-cos(_input_joint_angles[1])
          * (sin(_input_joint_angles[5]) * cos(_input_joint_angles[3])
              * cos(_input_joint_angles[4])
              + sin(_input_joint_angles[3]) * cos(_input_joint_angles[5]))
          + sin(_input_joint_angles[4]) * sin(_input_joint_angles[1])
              * sin(_input_joint_angles[5]))
      - sin(_input_joint_angles[0])
          * (-sin(_input_joint_angles[5]) * cos(_input_joint_angles[4])
              * sin(_input_joint_angles[3])
              + cos(_input_joint_angles[3]) * cos(_input_joint_angles[5]));
  final_transformation_matrix(1, 1) = sin(_input_joint_angles[0])
      * (-cos(_input_joint_angles[1])
          * (sin(_input_joint_angles[5]) * cos(_input_joint_angles[3])
              * cos(_input_joint_angles[4])
              + sin(_input_joint_angles[3]) * cos(_input_joint_angles[5]))
          + sin(_input_joint_angles[4]) * sin(_input_joint_angles[1])
              * sin(_input_joint_angles[5]))
      + cos(_input_joint_angles[0])
          * (-sin(_input_joint_angles[5]) * cos(_input_joint_angles[4])
              * sin(_input_joint_angles[3])
              + cos(_input_joint_angles[3]) * cos(_input_joint_angles[5]));
  final_transformation_matrix(1, 2) = sin(_input_joint_angles[1])
      * sin(_input_joint_angles[5]) * cos(_input_joint_angles[3])
      * cos(_input_joint_angles[4])
      + sin(_input_joint_angles[4]) * cos(_input_joint_angles[1])
          * sin(_input_joint_angles[5])
      + cos(_input_joint_angles[5]) * sin(_input_joint_angles[1])
          * sin(_input_joint_angles[3]);
  ;
  final_transformation_matrix(1, 3) = 0;
  final_transformation_matrix(2, 0) = cos(_input_joint_angles[0])
      * (cos(_input_joint_angles[1]) * cos(_input_joint_angles[3])
          * sin(_input_joint_angles[4])
          + cos(_input_joint_angles[4]) * sin(_input_joint_angles[1]))
      - sin(_input_joint_angles[0]) * sin(_input_joint_angles[3])
          * sin(_input_joint_angles[4]);
  final_transformation_matrix(2, 1) = sin(_input_joint_angles[0])
      * (cos(_input_joint_angles[1]) * cos(_input_joint_angles[3])
          * sin(_input_joint_angles[4])
          + cos(_input_joint_angles[4]) * sin(_input_joint_angles[1]))
      + cos(_input_joint_angles[0]) * sin(_input_joint_angles[3])
          * sin(_input_joint_angles[4]);
  ;
  final_transformation_matrix(2, 2) = -sin(_input_joint_angles[1])
      * cos(_input_joint_angles[3]) * sin(_input_joint_angles[4])
      + cos(_input_joint_angles[4]) * cos(_input_joint_angles[1]);
  final_transformation_matrix(2, 3) = 0;
  final_transformation_matrix(3, 0) = cos(_input_joint_angles[0])
      * sin(_input_joint_angles[1]) * I.get_dh_d()[2]
      - sin(_input_joint_angles[0]) * I.get_dh_d()[1];
  final_transformation_matrix(3, 1) = sin(_input_joint_angles[0])
      * sin(_input_joint_angles[1]) * I.get_dh_d()[2]
      + cos(_input_joint_angles[0]) * I.get_dh_d()[1];
  final_transformation_matrix(3, 2) = I.get_dh_d()[2]
      * cos(_input_joint_angles[1]);
  final_transformation_matrix(3, 3) = 1;

  /* extracting X,Y,Z from the final_transformation_matrix. */
  // to store the end-effector(X,Y,Z) positions
  std::vector<double> end_effector_coordinates;
  /* Extracting the Rotation matrix from the transformation matrix and
   * return it to a vectorend_effector_pose.
   */
  std::vector<double> end_effector_pose;
  end_effector_pose.push_back(final_transformation_matrix(0, 0));
  end_effector_pose.push_back(final_transformation_matrix(0, 1));
  end_effector_pose.push_back(final_transformation_matrix(0, 2));
  end_effector_pose.push_back(final_transformation_matrix(1, 0));
  end_effector_pose.push_back(final_transformation_matrix(1, 1));
  end_effector_pose.push_back(final_transformation_matrix(1, 2));
  end_effector_pose.push_back(final_transformation_matrix(2, 0));
  end_effector_pose.push_back(final_transformation_matrix(2, 1));
  end_effector_pose.push_back(final_transformation_matrix(2, 2));

  end_effector_coordinates.push_back(final_transformation_matrix(3, 0));
  end_effector_coordinates.push_back(final_transformation_matrix(3, 1));
  end_effector_coordinates.push_back(final_transformation_matrix(3, 2));
  // setting the output_coordinates as end_effector_coordinates
  set_output_coordinates(end_effector_coordinates);
  //setting the current_pose as end_effector_pose
  set_current_pose(end_effector_pose);
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
