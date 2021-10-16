#include <bits/stdc++.h>
#include <iostream>

class Forward_kinematics {
   private:
	std::vector <double> input_joint_angles;
	std::vector <double> output_joint_coordinates;
	std::vector <double> output_joint_angles;
	std::vector <double> current_robot_pose;
   public:
   	std::vector<double>solve_FK(std::vector<double>, std::vector<double>);
		void set_output_coordinates(std::vector<double>);
		void set_output_angles(std::vector<double>);
		void set_input_angles(std::vector<double>);
		void set_current_pose(std::vector<double>);
		std::vector<double> get_output_coordinates();
		std::vector<double> get_output_angles();
		std::vector<double> get_current_pose();
		std::vector<double> get_intput_angles();
};
