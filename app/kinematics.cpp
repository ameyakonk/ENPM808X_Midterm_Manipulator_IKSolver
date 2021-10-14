#include "kinematics.hpp"
#include <cstdlib>
#include <cmath>

using std::cout;
using std::endl;

std::vector<double> Kinematics::solve_IK(std::vector<double> input_joint_coordinates, std::vector<double> input_joint_angles) {
        
	double* theta = (double *)malloc(sizeof(double)*6);
	
	std::vector <double>::size_type i=1;
	
	double s = pow(pow((input_joint_coordinates[i+1] - dh_d[i-1]),2), 0.5);
	
	double r = pow(pow(input_joint_coordinates[i-1], 2) + pow(input_joint_coordinates[i],2), 0.5);

	double D = (pow(s, 2) + pow(r, 2) - pow(dh_a[i+1], 2) - pow(dh_a[i+2], 2))/(2*dh_a[i+1]*dh_a[i+2]);
       
       	theta[0] = atan2(input_joint_coordinates[i], input_joint_coordinates[i-1]);                                         /// need to define math.h in kinematics.hpp
        
	theta[2] = atan2(pow(1-pow(D, 2), 0.5),D);
	
	theta[1] = atan2(s, r) - atan2(dh_a[i+2]*sin(theta[2]), dh_a[i+1] + dh_a[i+2]*cos(theta[2])); 
        
	theta[3] = atan2(-(cos(theta[0])*sin(theta[1] + theta[2])*input_joint_angles[i+1]) - 
		(sin(theta[0])*sin(theta[1] + theta[2])*input_joint_angles[i+1]) + 
		(cos(theta[1] +theta[2])*input_joint_angles[i+7]), 
		(cos(theta[0])*cos(theta[1] + theta[2])*input_joint_angles[i+1]) + 
		(sin(theta[0])*cos(theta[1] + theta[2])*input_joint_angles[i+4]) +
		(sin(theta[1] + theta[2])* input_joint_angles[i+7])); 

        theta[4] = atan2(pow(1- pow(sin(theta[0])*input_joint_angles[i+1] - cos(theta[0])*input_joint_angles[i+4],2) , 0.5),
			(sin(theta[0])*input_joint_angles[i+1] - cos(theta[0])*input_joint_angles[i+4]));
        
	theta[5] = atan2( (sin(theta[0])*input_joint_angles[i]) - 
			  (cos(theta[0])*input_joint_angles[i+3]), 
			  -(sin(theta[0])*input_joint_angles[i-1]) +
			  (cos(theta[0])*input_joint_angles[i+2]));

	std::vector <double> temp_joint_output_angles;
        for(int i=5; i >= 0 ; i--) {
                temp_joint_output_angles.push_back(theta[i]);
        }
	delete(theta);
	return temp_joint_output_angles;
}

std::vector<double> Kinematics:: solve_FK(std::vector<double> output_joint_coordinates) {
 return output_joint_coordinates;
}

void Kinematics::set_input_coordinates(std::vector<double> _input_joint_coordinates) {
  input_joint_coordinates = _input_joint_coordinates;
}

void Kinematics::set_output_coordinates(std::vector<double> _output_joint_coordinates) {
  output_joint_coordinates = _output_joint_coordinates;
}

void Kinematics::set_input_angles(std::vector<double> _input_joint_angles) {
  input_joint_angles = _input_joint_angles;
}

void Kinematics::set_current_pose(std::vector<double> _current_robot_pose) {
  current_robot_pose = _current_robot_pose;
}

std::vector<double> Kinematics::get_input_coordinates() {
 return input_joint_coordinates;
}

std::vector<double> Kinematics::get_output_coordinates() {
 return output_joint_coordinates;
}

std::vector<double> Kinematics::get_input_angles() {
 return input_joint_angles;
}

std::vector<double> Kinematics::get_current_pose() {
 return current_robot_pose;
}

std::vector<double> Kinematics::get_dh_a() {
 return dh_a;
}

std::vector<double> Kinematics::get_dh_d() {
 return dh_d;
}

std::vector<double> Kinematics::get_dh_alpha() {
 return dh_alpha;
}


