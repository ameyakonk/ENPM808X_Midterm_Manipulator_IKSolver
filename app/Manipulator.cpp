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
 * @file /app/Manipulator.cpp
 * @author Rahul Karanam , Ameya Konkar
 * @copyright BSD 3-Clause License
 *
 * @brief This file contains the Forward Kinematics methods used to find
 *        out the end-effector coordinates of the robotic manipulator.
 *
 *
 */
// Header Files
#include "Manipulator.hpp"
void Manipulator::print_IK_solver() {
  // Instantiating Inverse_kinematics class
    Inverse_Kinematics I;
    // Instantiating Forward kinematics class
    Forward_Kinematics F;
    std::vector<double> ::size_type l = 0;
    std::vector<double> current_pose_1 {-0.66, 0.43, 0.61,
      0.73, 0.21, 0.64, 0.149,
        0.87, -0.46 };
    std::vector<double> current_pose_2 {-0.88, -0.17, 0.43, 0.45,
      -0.15, 0.87, -0.08, 0.97, 0.21};
    std::vector<double> current_pose_3  {-0.9, -0.01, 0.3, 0.33,
      -0.67, 0.66, 0.28, 0.7, 0.61};
    std::vector<double> current_pose_4 {-0.99, -0.12, 0.055, -0.12,
      0.74, 0.66, -0.04, 0.66, -0.74};
    std::vector<std::vector<double>> current_pose{current_pose_1,
      current_pose_2, current_pose_3, current_pose_4};
    I.set_dh_d({ 0, 5, 10, 0, 0, 0 });
    I.set_dh_a({ 0, 0, 0, 0, 0, 0 });
    I.set_dh_alpha({ -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });
    std::vector<double> coord_1{ 5, 8.66, 5 };
    std::vector<double> coord_2{0.001, 7.06, 8.66};
    std::vector<double> coord_3{3.62, 7.8, 7};
    std::vector<double> coord_4{2.6, 9.6, 5};
    std::vector<std::vector<double>> target_coordinates{coord_1,
      coord_2, coord_3, coord_4};
  std::cout<< "*******************"
      <<"*******************"<<
            "********************************************************"
        << std::endl;
    std::cout<< "                INVERSE KINEMATICS SOLVER FOR "
            <<"A 6 DOF ROBOTIC ARM - ACME ROBOTICS             "
        << std::endl;
    std::cout
        << "************************************************"<<
            "**********************************************"
        << std::endl;
    std::cout
        << "***TARGET COORDINATES  ***************   TARGET POSE  *****"
           << "****************************"
        << std::endl;
    std::cout
        << "************************************************"
          <<  "**********************************************"
        << std::endl;
    std::cout << "1." << std::setw(10) << "{5,8.66,5}" <<
         std::setw(30)<< "\t"
        << "{-0.66,0.43,0.61,0.73,0.21,0.64,"
           << "0.149,0.87,-0.46}" << std::endl;
    std::cout << "2." << std::setw(10) << "{0.001,7.06,"
        <<"8.66}" << std::setw(30)
        << "\t "<< "{-0.88,-0.17,0.43,0.45,-0.15,0.87,"
           << "-0.08,0.97,0.21}"<< std::endl;
    std::cout << "3." << std::setw(10) << "{3.62,"
        <<"7.8,7}" << std::setw(30)<< "\t"
        << "{-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,"
          <<  "0.7,0.61}" << std::endl;
    std::cout << "4." << std::setw(10) << "{2.6,9."
        <<"6,5}" << std::setw(30)<< "\t"
        << "{-0.99,-0.12,0.055,-0.12,0.74,0.66,"
          <<  "-0.04,0.66,-0.74}" << std::endl;
    std::cout
        << "************************************************************"
          <<  "**********************************"
        << std::endl;
    std::cout
        << "*************************************************************"
          <<  "*********************************"
        << std::endl;
    std::cout
        << "                             INVERSE KINEMATICS SOLVER         "
          <<  "                               "
        << std::endl;
    std::cout
        << "****************************************************"
         <<   "******************************************"
        << std::endl;
    std::cout
        << "*******************          END EFFECTOR JOINT ANGLES      "
          <<  " ******************"
          <<  "***************"
        << std::endl;
    std::cout
        << "**************************************"
           << "*******************************"
           << "*************************"
        << std::endl;
    for (l = 0; l < 4; l++) {
      std::cout<< "***************TARGET-"<< l+1 <<"******"
         << "**************************"
          << "*************************"<< std::endl;
        I.set_input_coordinates(target_coordinates[l]);
        I.solve_IK(I.get_input_coordinates(), current_pose[l]);
        for (int j = 0; j < 6; j++) {
          std::cout << "Theta- "<< j<< std::setw(30)<<
               I.get_output_angles()[j]<< std::endl;
        }
     }
}
