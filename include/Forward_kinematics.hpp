#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include "Eigen/Core"
#define PI 3.14

class Forward_Kinematics {
   private:
		std::vector <double> input_joint_angles;
		std::vector <double> output_joint_coordinates;
		std::vector <double> output_joint_angles;
		std::vector <double> current_robot_pose;
   public:
   	void solve_FK(std::vector<double>);
   	std::vector<double> create_transformation_matrix();
		void set_output_coordinates(std::vector<double>);
		void set_output_angles(std::vector<double>);
		void set_input_angles(std::vector<double>);
		void set_current_pose(std::vector<double>);
		std::vector<double> get_output_coordinates();
		std::vector<double> get_output_angles();
		std::vector<double> get_current_pose();
		std::vector<double> get_input_angles();
};

#endif