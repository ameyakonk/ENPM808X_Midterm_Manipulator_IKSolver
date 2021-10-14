#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#define PI 3.14

class Kinematics {
   private:
	   std::vector <double> input_joint_angles;
	   std::vector <double> input_joint_coordinates;
	   std::vector <double> output_joint_coordinates;
	   std::vector <double> output_joint_angles;
	   std::vector <double> current_robot_pose;
	   std::vector <double> link_lengths;
	   std::vector <double> joint_angle_constraints;
	   std::vector <double> dh_a {0, 10, 20};
	   std::vector <double> dh_d {10, 0, 0};
	   std::vector <double> dh_alpha {PI/2, 0, 0};
	   std::vector <double> output_bias { };
   public:
	   std::vector<double> solve_IK(std::vector<double> , std::vector<double>);
	   std::vector<double> solve_FK(std::vector<double>);
	   void set_input_coordinates(std::vector<double>);
	   void set_output_coordinates(std::vector<double>);  
	   void set_input_angles(std::vector<double>);
	   void set_current_pose(std::vector<double>);
	   std::vector<double> get_input_coordinates();
	   std::vector<double> get_output_coordinates();
           std::vector<double> get_input_angles();
	   std::vector<double> get_current_pose();
	   std::vector<double> set_output_coordinates();
	   std::vector<double> get_dh_a();
           std::vector<double> get_dh_d();
	   std::vector<double> get_dh_alpha();
};
#endif
