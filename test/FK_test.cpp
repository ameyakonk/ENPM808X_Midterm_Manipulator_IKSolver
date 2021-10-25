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
  F.set_output_coordinates({ 20, 20, 20 });
  std::vector<double> _output_coordinates({ 20, 20, 20 });
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
  F.set_output_angles({ PI / 2, PI / 4, PI / 6 });
  std::vector<double> _output_angles({ PI / 2, PI / 4, PI / 6 });
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
  F.set_current_pose({ PI / 2, PI / 4, PI / 6, PI / 2, PI / 4, PI / 6 });
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
  F.set_input_angles({ PI / 2, PI / 4, PI / 6 });
  std::vector<double> _input_angles({ PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 2; i++)
    ASSERT_EQ(_input_angles[i], F.get_input_angles()[i]);
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK. 
 *       It checks whether the end effector coordinates({PI / 6, PI / 3, 0, PI / 4,1.30, PI /2 }) match with the output_coordinates.
 * @brief Google Unit Test to check for end-effector coordinates from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(solve_FK_1, should_return_output_coordinates) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 6, PI / 3, 0, PI / 4, 1.30, PI /2 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_coord{5, 8.66, 5};
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_coord[i]-0.08)
                && (F.get_output_coordinates()[i] <= end_coord[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK. 
 *       It checks whether the end effector coordinates({PI / 4, PI / 6, 0, PI / 3,PI/3, PI /2 }) match with the output_coordinates.
 * @brief Google Unit Test to check for end-effector coordinates from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(solve_FK_2, should_return_output_coordinates) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 4, PI / 6, 0, PI / 3, PI/3, PI /2 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_coord{0.001, 7.06, 8.66};
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_coord[i]-0.08)
                && (F.get_output_coordinates()[i] <= end_coord[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK. 
 *       It checks whether the end effector coordinates({PI / 6, PI / 4, 0, PI / 2,PI/6, PI /3 }) match with the output_coordinates.
 * @brief Google Unit Test to check for end-effector coordinates from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(solve_FK_3, should_return_output_coordinates) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 6, PI / 4, 0, PI / 2, PI/6, PI /3 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_coord{3.62, 7.8, 7};
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_coord[i]-0.08)
                && (F.get_output_coordinates()[i] <= end_coord[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK. 
 *       It checks whether the end effector coordinates({PI / 4, PI / 3, 0, PI / 6,PI/2, PI /4 }) match with the output_coordinates.
 * @brief Google Unit Test to check for end-effector coordinates from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(solve_FK_4, should_return_output_coordinates) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 4, PI / 3, 0, PI / 6, PI/2, PI /4 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_coord{2.6, 9.6, 5};
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_coord[i]-0.08)
                && (F.get_output_coordinates()[i] <= end_coord[i]+0.08));
}


/**
 * @test This test checks for the Forward Kinematics method solve_FK for the end effector pose. 
 *       It checks whether the end effector pose({-0.66,0.43,0.61,0.73,0.21,0.64,0.149,0.87,-0.46 }) match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(get_current_pose_FK_1, should_return_output_pose) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 6, PI / 3, 0, PI / 4, 1.30, PI /2 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_pose{-0.66, 0.43, 0.61,
    0.73, 0.21, 0.64, 0.149, 0.87, -0.46};
  for (i = 0; i < 9; i++)
    EXPECT_TRUE((F.get_current_pose()[i] >= end_pose[i]-0.08)
                && (F.get_current_pose()[i] <= end_pose[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK for the end effector pose. 
 *       It checks whether the end effector pose({{-0.88,-0.17,0.43,0.45,-0.15,0.87,-0.08,0.97,0.21}) match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(get_current_pose_FK_2, should_return_output_cpose) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 4, PI / 6, 0, PI / 3, PI/3, PI /2 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_pose{-0.88, -0.17, 0.43, 0.45,
    -0.15, 0.87, -0.08, 0.97, 0.21};
  for (i = 0; i < 9; i++)
    EXPECT_TRUE((F.get_current_pose()[i] >= end_pose[i]-0.08)
                && (F.get_current_pose()[i] <= end_pose[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK for the end effector pose. 
 *       It checks whether the end effector pose({-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,0.7,0.61}) match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(get_current_pose_FK_3, should_return_output_pose) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 6, PI / 4, 0, PI / 2, PI/6, PI /3 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_pose{-0.9 , -0.01, 0.3, 0.33,
     -0.67, 0.66, 0.28, 0.7, 0.61};
  for (i = 0; i < 9; i++)
    EXPECT_TRUE((F.get_current_pose()[i] >= end_pose[i]-0.08)
                && (F.get_current_pose()[i] <= end_pose[i]+0.08));
}
/**
 * @test This test checks for the Forward Kinematics method solve_FK for the end effector pose. 
 *       It checks whether the end effector pose({-0.99,-0.12,0.075,-0.12,0.74,0.66,-0.04,0.66,-0.74}) match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method and checking for the output bias of 0.08.
 */
TEST(get_current_pose_FK_4, should_return_output_pose) {
  Forward_Kinematics F;
  std::vector<double> end {PI / 4, PI / 3, 0, PI / 6, PI/2, PI /4 };
  std::vector<double>::size_type i = 0;
  F.solve_FK(end);
  std::vector<double> end_pose{-0.99, -0.12, 0.055,
    -0.12, 0.74, 0.66, -0.04, 0.66, -0.74};
  for (i = 0; i < 9; i++)
    EXPECT_TRUE((F.get_current_pose()[i] >= end_pose[i]-0.08)
                && (F.get_current_pose()[i] <= end_pose[i]+0.08));
}
