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
 * @file FK_test.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This file contains the all the google unit tests for testing
 *        the methods in the Forward_kinematics.cpp
 *
 *
 */

// Header Files
#include <gtest/gtest.h>
#include "Forward_kinematics.hpp"
#define PI 3.14

/**
 * @test This test checks for get and set methods for the output_coordinates.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_output_coordinates_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i;
  F.set_output_coordinates( { 20, 20, 20 });
  std::vector<double> _output_coordinates( { 20, 20, 20 });
  for (i = 0; i < 3; i++)
    ASSERT_EQ(_output_coordinates[i], F.get_output_coordinates()[i]);
}
/**
 * @test This test checks for get and set methods for the output_angles.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_output_angles_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i = 0;
  F.set_output_angles( { PI / 2, PI / 4, PI / 6 });
  std::vector<double> _output_angles( { PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 2; i++)
    ASSERT_EQ(_output_angles[i], F.get_output_angles()[i]);
}
/**
 * @test This test checks for get and set methods for the current_robot_pose.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_current_pose_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i = 0;
  F.set_current_pose( { PI / 2, PI / 4, PI / 6, PI / 2, PI / 4, PI / 6 });
  std::vector<double> _current_pose(
      { PI / 2, PI / 4, PI / 6, PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 6; i++)
    ASSERT_EQ(_current_pose[i], F.get_current_pose()[i]);
}
/**
 * @test This test checks for get and set methods for the input_angles.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_input_angles_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i = 0;
  F.set_input_angles( { PI / 2, PI / 4, PI / 6 });
  std::vector<double> _input_angles( { PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 2; i++)
    ASSERT_EQ(_input_angles[i], F.get_input_angles()[i]);
}

