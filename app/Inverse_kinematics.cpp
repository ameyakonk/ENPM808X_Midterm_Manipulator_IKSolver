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
 * @file Inverse_kinematics.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This file contains the Forward Kinematics methods used to find
 *        out the end-effector coordinates of the robotic manipulator.
 *
 *
 */

#include "Inverse_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"

using std::cout;
using std::endl;

/**
 * @fn void convert_input_angles_to_rotation_matrix(std::vector<double>)
 * @brief This Function will compute the rotation matrix based on the angles
 *        Pitch, Yaw, Roll.
 *
 * @param _input_joint_angles is the desired end-effector angles.
 * @return rotation_matrix
 */

std::vector<double> Inverse_Kinematics::convert_input_angles_to_rotation_matrix(
    const std::vector<double> &input_joint_angles) {
  std::vector<double>::size_type i = 1;
  /*  Creating Eigen Matrices for creating rotation matrices for roll,
   *  pitch, yaw
   */
  Eigen::Matrix<double, 3, 3> ROLL;
  ROLL << 1.0f, 0.0f, 0.0f, 0.0f, cos(input_joint_angles[i - 1]), (-sin(
      input_joint_angles[i - 1])), 0.0f, sin(input_joint_angles[i - 1]), cos(
      input_joint_angles[i - 1]);
  Eigen::Matrix<double, 3, 3> PITCH;
  PITCH << cos(input_joint_angles[i]), 0.0f, sin(input_joint_angles[i]), 0.0f, 1.0f, 0.0f, (-sin(
      input_joint_angles[i])), 0.0f, cos(input_joint_angles[i]);
  Eigen::Matrix<double, 3, 3> YAW;
  YAW << cos(input_joint_angles[i + 1]), (-sin(input_joint_angles[i + 1])), 0.0f, sin(
      input_joint_angles[i + 1]), cos(input_joint_angles[i + 1]), 0.0f, 0.0f, 0.0f, 1.0f;
  Eigen::Matrix<double, 3, 3> ROTATION_MATRIX;
  ROTATION_MATRIX = YAW * PITCH * ROLL;
  std::vector<double> rotation_matrix;
  for (int r = 0; r < 9; r++)
    rotation_matrix.push_back(ROTATION_MATRIX(r));
  /*  Setting input parameters */
  set_input_angles(rotation_matrix);
  set_input_coordinates( { 5, 8.6, 5 });
  set_dh_d( { 0, 5, 10, 0, 0, 0 });
  set_dh_a( { 0, 0, 0, 0, 0, 0 });
  set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  solve_IK(get_input_coordinates(), get_input_angles());
  return rotation_matrix;
}

/**
 * @fn void solve_IK(std::vector<double>)
 * @brief This Function will compute the InverseKinematics
 *        for the given input end effector position coordinates and angles
 *        and sets the end effector coordinates to output_joint angles.
 *
 * @param _input_joint_angles is the end effector angles
 * @param _input_joint_coordinates is the end effector coordinates
 * @return None
 */

void Inverse_Kinematics::solve_IK(
    const std::vector<double> &input_joint_coordinates,
    const std::vector<double> &input_joint_angles) {
  /*  Calculating final joint angles */
  double *theta = reinterpret_cast<double*>(malloc(sizeof(double) * 7));
  std::vector<double>::size_type i = 1;
  double r = pow(
      pow(input_joint_coordinates[i - 1], 2)
          + pow(input_joint_coordinates[i], 2),
      0.5);
  theta[0] = atan(input_joint_coordinates[i] / input_joint_coordinates[i - 1])
      - atan(dh_d[i] / (pow(pow(r, 2) - pow(dh_d[i], 2), 0.5)));
  theta[1] = atan(
      ((cos(theta[0]) * input_joint_coordinates[i - 1])
          + (sin(theta[0]) * input_joint_coordinates[i]))
          / input_joint_coordinates[i + 1]);
  
  double d3 = (input_joint_coordinates[i - 1] * cos(theta[0]))
      + (sin(theta[0]) * input_joint_coordinates[i]) * sin(theta[1])
      + (input_joint_coordinates[i + 1] * cos(theta[1]));
  theta[3] = atan(
      ((-sin(theta[0])) * input_joint_angles[i + 5]
          + cos(theta[0]) * input_joint_angles[i + 6])
          / ((cos(theta[1])
              * (cos(theta[0]) * input_joint_angles[i + 5]
                  + sin(theta[0]) * input_joint_angles[i + 6])
              - sin(theta[1]) * input_joint_angles[i + 7])));
  double I = cos(theta[0]) * input_joint_angles[i + 2]
      + sin(theta[0]) * input_joint_angles[i + 3];
  double n = (-sin(theta[0])) * input_joint_angles[i + 2]
      + cos(theta[0]) * input_joint_angles[i + 3];
  theta[4] = atan(
      (cos(theta[3])
          * (cos(theta[1])
              * (cos(theta[0]) * input_joint_angles[i + 5]
                  + sin(theta[0]) * input_joint_angles[i + 6])
              - sin(theta[1]) * input_joint_angles[i + 7])
          + sin(theta[3])
              * (cos(theta[0]) * input_joint_angles[i + 6]
                  - sin(theta[0]) * input_joint_angles[i + 5]))
          / (sin(theta[1])
              * (cos(theta[0]) * input_joint_angles[i + 5]
                  + sin(theta[0]) * input_joint_angles[i + 6])
              + cos(theta[1]) * input_joint_angles[i + 7]));
  theta[5] = atan(
      ((-cos(theta[4]))
          * (cos(theta[3])
              * (cos(theta[1]) * I - sin(theta[1]) * input_joint_angles[i + 4])
              + sin(theta[3]) * n)
          + sin(theta[4])
              * (sin(theta[1]) * I + cos(theta[1]) * input_joint_angles[i + 4]))
          / (-sin(theta[3])
              * (cos(theta[1]) * I - sin(theta[1]) * input_joint_angles[i + 4])
              + cos(theta[3]) * n));
  std::vector<double> output_joint_angles;
  std::vector<double> _dh_d { 0, 5, d3, 0, 0, 0 };
  set_dh_d(_dh_d);
  for (int k = 0; k < 6; k++)
    output_joint_angles.push_back(fabs(theta[k]));
  free(theta);
  set_output_angles(output_joint_angles);
}

/**
 * @fn void set_input_coordinates(std::vector<double>)
 * @brief It sets the given input to input_joint_coordinates
 *
 * @param _input_joint_coordinates
 * @return None
 */

void Inverse_Kinematics::set_input_coordinates(
    const std::vector<double> &_input_joint_coordinates) {
  input_joint_coordinates = _input_joint_coordinates;
}

/**
 * @fn std::vector<double> get_input_coordinates()
 * @brief Getter method for returning the input_joint_coordinates
 *
 * @return input_joint_coordinates
 */

std::vector<double> Inverse_Kinematics::get_input_coordinates() {
  return input_joint_coordinates;
}

/**
 * @fn void set_input_angles(std::vector<double>)
 * @brief It sets the given input to input_joint_angles
 *
 * @param _input_joint_angles
 * @return None
 */

void Inverse_Kinematics::set_input_angles(
    const std::vector<double> &_input_joint_angles) {
  input_joint_angles = _input_joint_angles;
}

/**
 * @fn std::vector<double> get_input_angles()
 * @brief Getter method for returning the input_joint_angles
 *
 * @return input_joint_angles
 */

std::vector<double> Inverse_Kinematics::get_input_angles() {
  return input_joint_angles;
}

/**
 * @fn void set_output_angles(std::vector<double>)
 * @brief It sets the given input to output_joint_angles
 *
 * @param _output_joint_angles
 * @return None
 */

void Inverse_Kinematics::set_output_angles(
    const std::vector<double> &_output_joint_angles) {
  output_joint_angles = _output_joint_angles;
}

/**
 * @fn std::vector<double> output_joint_angles()
 * @brief Getter method for returning the output_joint_angles
 *
 * @return output_joint_angles
 */

std::vector<double> Inverse_Kinematics::get_output_angles() {
  return output_joint_angles;
}

/**
 * @fn void set_output_coordinates(std::vector<double>)
 * @brief It sets the given input to output_joint_coordinates
 *
 * @param _output_joint_coordinates
 * @return None
 */

void Inverse_Kinematics::set_output_coordinates(
    const std::vector<double> &_output_joint_coordinates) {
  output_joint_coordinates = _output_joint_coordinates;
}

/**
 * @fn std::vector<double> get_output_coordinates()
 * @brief Getter method for returning the get_output_coordinates
 *
 * @return get_output_coordinates
 */

std::vector<double> Inverse_Kinematics::get_output_coordinates() {
  return output_joint_coordinates;
}

/**
 * @fn void set_current_pose(std::vector<double>)
 * @brief It sets the given input to set_current_pose
 *
 * @param _set_current_pose
 * @return None
 */

void Inverse_Kinematics::set_current_pose(
    const std::vector<double> &_current_robot_pose) {
  current_robot_pose = _current_robot_pose;
}

/**
 * @fn std::vector<double> get_current_pose()
 * @brief Getter method for returning the current_robot_pose
 *
 * @return current_robot_pose
 */

std::vector<double> Inverse_Kinematics::get_current_pose() {
  return current_robot_pose;
}

/**
 * @fn void set_dh_a(std::vector<double>)
 * @brief It sets the given input to set_dh_a
 *
 * @param _set_dh_a
 * @return None
 */

void Inverse_Kinematics::set_dh_a(const std::vector<double> &_dh_a) {
  dh_a = _dh_a;
}

/**
 * @fn std::vector<double> get_dh_a()
 * @brief Getter method for returning the get_dh_a
 *
 * @return dh_a
 */

std::vector<double> Inverse_Kinematics::get_dh_a() {
  return dh_a;
}

/**
 * @fn void set_dh_d(std::vector<double>)
 * @brief It sets the given input to _dh_d
 *
 * @param dh_d
 * @return None
 */

void Inverse_Kinematics::set_dh_d(const std::vector<double> &_dh_d) {
  dh_d = _dh_d;
}

/**
 * @fn std::vector<double> get_dh_d()
 * @brief Getter method for returning the dh_d
 *
 * @return dh_d
 */

std::vector<double> Inverse_Kinematics::get_dh_d() {
  return dh_d;
}

/**
 * @fn void set_dh_alpha(std::vector<double>)
 * @brief It sets the given input to dh_alpha
 *
 * @param _dh_alpha
 * @return None
 */

void Inverse_Kinematics::set_dh_alpha(const std::vector<double> &_dh_alpha) {
  dh_alpha = _dh_alpha;
}

/**
 * @fn std::vector<double> get_dh_alpha()
 * @brief Getter method for returning the current_robot_pose
 *
 * @return dh_alpha
 */

std::vector<double> Inverse_Kinematics::get_dh_alpha() {
  return dh_alpha;
}

/**
 * @fn void reset_pose()
 * @brief It resets the position of the manipulator
 *
 * @return None
 */

void Inverse_Kinematics::reset_pose() {
}
