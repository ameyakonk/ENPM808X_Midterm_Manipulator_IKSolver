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
#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include "Inverse_kinematics.hpp"
#include "Forward_kinematics.hpp"
#include "matplotlibcpp.h"
#include <cmath>
namespace plt = matplotlibcpp;
//namespace plt = matplotlibcpp;
#define PI 3.14
using std::cout;
using std::endl;
using namespace Eigen;

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
	//std::vector<double> end {PI / 4, PI / 6, 0, PI / 3, PI / 3, PI /2 };
    std::vector<double> current_pose {-0.66,0.43,0.61,0.73,0.21,0.64,0.149,0.87,-0.46};
    //std::vector<double> current_pose {-0.88,-0.17,0.43,0.45,-0.15,0.87,-0.08,0.97,0.21};
    //std::vector<double> current_pose {-0.9,-0.01,0.3,0.33,-0.67,0.66,0.28,0.7,0.61};
    //std::vector<double> current_pose {-0.99,-0.12,0.055,-0.12,0.74,0.66,-0.04,0.66,-0.74};

    I.set_dh_d( { 0, 5, 10, 0, 0, 0 });
    I.set_dh_a( { 0, 0, 0, 0, 0, 0 });
    I.set_dh_alpha( { -PI / 2, PI / 2, 0, (-PI / 2), PI / 2, 0 });

    I.set_input_coordinates({5,8.66,5});
    //I.set_input_coordinates({0.001,7.06,8.66});
    //I.set_input_coordinates({3.62,7.8,7});
    //I.set_input_coordinates({2.6,9.6,5});

	//F.solve_FK(end);
    I.solve_IK(I.get_input_coordinates(),current_pose);
    //std::vector<double> tet;
    F.solve_FK(I.get_output_angles());
	for (int j = 0; j < 3; j++) {
	 	std::cout << "Angles"<<F.get_output_coordinates()[j] << std::endl;
        

  
	}
 //    for (int k=0;k<9;k++){
 //        std::cout<<"rPY"<<F.get_current_pose()[k]<<std::endl;
 //    }
    

    for(int i=0;i<6;i++){
        std::cout<<I.get_output_angles()[i]<<std::endl;
     }


}

