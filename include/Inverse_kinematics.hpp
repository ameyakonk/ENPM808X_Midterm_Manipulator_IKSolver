#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include "Eigen/Core"
#define PI 3.14

class Inverse_Kinematics {
   private:
	   std::vector <double> input_joint_angles;
	   std::vector <double> input_joint_coordinates;
	   std::vector <double> output_joint_coordinates;
	   std::vector <double> output_joint_angles;
	   std::vector <double> current_robot_pose;
	   std::vector <double> link_lengths;
	   std::vector <double> joint_angle_constraints;
	   std::vector <double> dh_a;
	   std::vector <double> dh_d;
	   std::vector <double> dh_alpha;
	   std::vector <double> output_bias;
   public:
	   void solve_IK(std::vector<double> , std::vector<double>);
	   void set_input_coordinates(std::vector<double>);
	   void set_output_coordinates(std::vector<double>); 
	   void set_output_angles(std::vector<double>);  
	   void set_input_angles(std::vector<double>);
	   void set_current_pose(std::vector<double>);
	   void set_dh_a(std::vector<double>);
	   void set_dh_d(std::vector<double>);
	   void set_dh_alpha(std::vector<double>); 
	   std::vector<double> get_input_coordinates();
	   std::vector<double> get_output_coordinates();
      std::vector<double> get_input_angles();
      std::vector<double> get_output_angles();
      std::vector<double> get_current_pose();
	   std::vector<double> set_output_coordinates();
	   std::vector<double> get_dh_a();
      std::vector<double> get_dh_d();
	   std::vector<double> get_dh_alpha();
	   void reset_pose();
	   void convert_input_angles_to_rotation_matrix(std::vector<double>);
};
#endif
