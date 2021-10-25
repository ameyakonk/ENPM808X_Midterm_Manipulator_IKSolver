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
 * @file IK_test.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief @brief This file contains the all the google unit tests for testing
 *        the methods in the Inverse_kinematics.cpp
 *
 */

#include <gtest/gtest.h>
#include "Inverse_kinematics.hpp"
#include "Forward_kinematics.hpp"
#define PI 3.14

/**
 * @test This test checks for get and set methods for the input_coordinates.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_input_coordinates_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i;
  I.set_input_coordinates({20, 20, 20});
  std::vector <double> _input_coordinates({20, 20, 20});
  for (i = 0 ; i < 3; i++)
    ASSERT_EQ(_input_coordinates[i], I.get_input_coordinates()[i]);
}

/**
 * @test This test checks for get and set methods for the input_angles.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_input_angles_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_input_angles({PI/2, PI/4, PI/6});
  std::vector <double> _input_angles({PI/2, PI/4, PI/6});
  for (i = 0 ; i < 2; i++)ASSERT_EQ(_input_angles[i], I.get_input_angles()[i]);
}

/**
 * @test This test checks for get and set methods for the output_angles.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_output_angles_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_output_angles({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _output_angles({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for (i = 0 ; i < 6; i++)
    ASSERT_EQ(_output_angles[i], I.get_output_angles()[i]);
}

/**
 * @test This test checks for get and set methods for the current_pose.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_current_pose_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_current_pose({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _current_pose({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for (i = 0 ; i < 6; i++)ASSERT_EQ(_current_pose[i], I.get_current_pose()[i]);
}

/**
 * @test This test checks for get and set methods for the dh_a.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_dh_a_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_dh_a({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _dh_a({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for (i = 0 ; i < 6; i++)ASSERT_EQ(_dh_a[i], I.get_dh_a()[i]);
}

/**
 * @test This test checks for get and set methods for the dh_d
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_dh_d_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_dh_d({0, 5, 10, 0, 0, 0});
  std::vector <double> _dh_d({0, 5, 10, 0, 0, 0});
  for (i = 0 ; i < 6; i++)ASSERT_EQ(_dh_d[i], I.get_dh_d()[i]);
}

/**
 * @test This test checks for get and set methods for the dh_alpha.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_dh_alpha_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.set_dh_alpha({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _dh_alpha({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for (i = 0 ; i < 6; i++)ASSERT_EQ(_dh_alpha[i], I.get_dh_alpha()[i]);
}

/**
 * @test This test checks for get and set methods for 
 *       the convert_input_angles_to_rotation_matrix.
 * @brief Google Unit Test for getter and setter methods.
 */
TEST(get_convert_input_angles_to_rotation_matrix_IK, should_return_set_values) {
  Inverse_Kinematics I;
  std::vector <double>::size_type i = 0;
  I.convert_input_angles_to_rotation_matrix({PI/2, PI/4, PI/4});
  std::vector <double> _dh_alpha({-1.57, 1.57, 0, -1.57, 1.57, 0});
  for (i = 0 ; i < 6; i++)ASSERT_EQ(_dh_alpha[i], I.get_dh_alpha()[i]);
}

/**
 * @test This test checks for the Inverse Kinematics method solve_IK for the end effector pose. 
 *       It checks whether the end_pose{-0.66,0.43,0.61,0.73,0.21,0.64,0.149,0.87,-0.46} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_IK method and checking for the output bias of 0.08.
 */
TEST(get_output_angles_IK_1, should_return_output_angles) {
  Inverse_Kinematics I;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({5,8.66,5});
  std::vector<double> current_pose {-0.66,0.43,0.61,0.73,0.21,0.64,0.149,0.87,-0.46};
  std::vector<double>::size_type i = 0;
  I.solve_IK(I.get_input_coordinates(),current_pose);
  std::vector<double> end_pose{0.52,1.04,0,0.785,1.30,1.57};
  for (i = 0; i < 6; i++)
    EXPECT_TRUE((I.get_output_angles()[i] >= end_pose[i]-0.08) && (I.get_output_angles()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse Kinematics method solve_IK for the end effector pose. 
 *       It checks whether the end_pose{-0.88,-0.17,0.43,0.45,-0.15,0.87,-0.08,0.97,0.21} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_IK method and checking for the output bias of 0.08.
 */
TEST(get_output_angles_IK_2, should_return_output_angles) {
  Inverse_Kinematics I;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({0.001,7.06,8.66});
  std::vector<double> current_pose {-0.88,-0.17,0.43,0.45,-0.15,0.87,-0.08,0.97,0.21};
  std::vector<double>::size_type i = 0;
  I.solve_IK(I.get_input_coordinates(),current_pose);
  std::vector<double> end_pose{0.785,0.52,0,1.046,1.046,1.57};
  for (i = 0; i < 6; i++)
    EXPECT_TRUE((I.get_output_angles()[i] >= end_pose[i]-0.08) && (I.get_output_angles()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse Kinematics method solve_IK for the end effector pose. 
 *       It checks whether the end_pose{-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,0.7,0.61} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_IK method and checking for the output bias of 0.08.
 */
TEST(get_output_angles_IK_3, should_return_output_angles) {
  Inverse_Kinematics I;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({3.62,7.8,7});
  std::vector<double> current_pose {-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,0.7,0.61};
  std::vector<double>::size_type i = 0;
  I.solve_IK(I.get_input_coordinates(),current_pose);
  std::vector<double> end_pose{0.52,0.785,0,1.57,0.5233,1.04};
  for (i = 0; i < 6; i++)
    EXPECT_TRUE((I.get_output_angles()[i] >= end_pose[i]-0.08) && (I.get_output_angles()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse Kinematics method solve_IK for the end effector pose. 
 *       It checks whether the end_pose{-0.99,-0.12,0.055,-0.12,0.74,0.66,-0.04,0.66,-0.74} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_IK method and checking for the output bias of 0.08.
 */
TEST(get_output_angles_IK_4, should_return_output_angles) {
  Inverse_Kinematics I;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({2.6,9.6,5});
  std::vector<double> current_pose {-0.99,-0.12,0.055,-0.12,0.74,0.66,-0.04,0.66,-0.74};
  std::vector<double>::size_type i = 0;
  I.solve_IK(I.get_input_coordinates(),current_pose);
  std::vector<double> end_pose{0.785,1.04,0,0.523,1.57,0.785};
  for (i = 0; i < 6; i++)
    EXPECT_TRUE((I.get_output_angles()[i] >= end_pose[i]-0.08) && (I.get_output_angles()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse and Forward Kinematics methods for the end effector pose. 
 *       It checks whether the end_pose{5,8.66,5} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method using input parameters
 *        from solve_IK parameters.
 */
TEST(FK_PLUS_IK_1, should_return_output_angles) {
  Inverse_Kinematics I;
  Forward_Kinematics F;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({5,8.66,5});
  
  std::vector<double> current_pose {-0.66,0.43,0.61,0.73,0.21,0.64,0.149,0.87,-0.46};
  std::vector<double>::size_type i = 0;
  
  I.solve_IK(I.get_input_coordinates(),current_pose);
  F.solve_FK(I.get_output_angles());
  
  std::vector<double> end_pose{5,8.66,5};
  
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_pose[i]-0.08) && (F.get_output_coordinates()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse and Forward Kinematics methods solve_IK for the end effector pose. 
 *       It checks whether the end_pose{0.001,7.06,8.66} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method using input parameters
 *        from solve_IK parameters.
 */ 
TEST(FK_PLUS_IK_2, should_return_output_angles) {
  Inverse_Kinematics I;
  Forward_Kinematics F;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({0.001,7.06,8.66});

  std::vector<double> current_pose {-0.88,-0.17,0.43,0.45,-0.15,0.87,-0.08,0.97,0.21};
  std::vector<double>::size_type i = 0;
  
  I.solve_IK(I.get_input_coordinates(),current_pose);
  F.solve_FK(I.get_output_angles());
  
  std::vector<double> end_pose{0.001,7.06,8.66};
  
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_pose[i]-0.08) && (F.get_output_coordinates()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse and Forward Kinematics methods for the end effector pose. 
 *       It checks whether the end_pose{3.62,7.8,7} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method using input parameters
 *        from solve_IK method parameters.
 */
TEST(FK_PLUS_IK_3, should_return_output_angles) {
  Inverse_Kinematics I;
  Forward_Kinematics F;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({3.62,7.8,7});

  std::vector<double> current_pose {-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,0.7,0.61};
  std::vector<double>::size_type i = 0;
  
  I.solve_IK(I.get_input_coordinates(),current_pose);
  F.solve_FK(I.get_output_angles());
  
  std::vector<double> end_pose{3.62,7.8,7};
  
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_pose[i]-0.08) && (F.get_output_coordinates()[i] <= end_pose[i]+0.08));
}

/**
 * @test This test checks for the Inverse and Forward Kinematics methods for the end effector pose. 
 *       It checks whether the end_pose{2.6,9.6,5} match with the current pose.
 * @brief Google Unit Test to check for end-effector pose from the solve_FK method using input parameters
 *        from solve_IK method parameters.
 */
TEST(FK_PLUS_IK_4, should_return_output_angles) {
  Inverse_Kinematics I;
  Forward_Kinematics F;
  I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
  I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
  I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
  I.set_input_coordinates({2.6,9.6,5});
  
  std::vector<double> current_pose {-0.99,-0.12,0.055,-0.12,0.74,0.66,-0.04,0.66,-0.74};
  std::vector<double>::size_type i = 0;
  
  I.solve_IK(I.get_input_coordinates(),current_pose);
  F.solve_FK(I.get_output_angles());
  
  std::vector<double> end_pose{2.6,9.6,5};
  
  for (i = 0; i < 3; i++)
    EXPECT_TRUE((F.get_output_coordinates()[i] >= end_pose[i]-0.08) && (F.get_output_coordinates()[i] <= end_pose[i]+0.08));
}