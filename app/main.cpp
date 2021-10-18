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
 * @file main.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This is our main source code file.
 *        It calls inverse Kinematics to implement our IK solver to
 *        simulate our path.
 *
 *
 */

// Header Files
#include <Eigen/Core>
#include <iostream>
#include "Inverse_kinematics.hpp"
#include "Forward_kinematics.hpp"
#define PI 3.14
using std::cout;
using std::endl;

/**
 * @fn int main()
 * @brief We use this main function to output the output
 * joint coordinates for the given input_coordinates
 *
 * @return 0;
 */
int main() {
  // Instantiating Inverse_kinematics class
  Inverse_Kinematics I;
  // Instantiating Forward kinematics class
  Forward_Kinematics F;
  std::vector<double> temp_input_joint_angles { PI / 2, PI / 4, PI / 4 };
  I.convert_input_angles_to_rotation_matrix(temp_input_joint_angles);
  F.solve_FK(I.get_output_angles());
  std::vector<double>::size_type i = 0;
  for (i = 0; i < 6; i++) {
    std::cout << I.get_output_angles()[i] << std::endl;
  }
  I.reset_pose();
}
