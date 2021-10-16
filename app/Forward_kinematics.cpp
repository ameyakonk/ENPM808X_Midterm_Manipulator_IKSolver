#include "Forward_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"

using std::cout;
using std::endl;

using namespace Eigen;

void Forward_Kinematics::solve_FK(std::vector<double> _input_joint_angles)
{
	
}

std::vector<double> Forward_Kinematics::create_transformation_matrix()
{

}

void Forward_Kinematics::set_output_coordinates(std::vector<double> _output_joint_coordinates)
{
	output_joint_coordinates = _output_joint_coordinates;
}

void Forward_Kinematics::set_output_angles(std::vector<double> _output_joint_angles)
{
	output_joint_angles = _output_joint_angles;
}

void Forward_Kinematics::set_input_angles(std::vector<double> _input_joint_angles)
{
	input_joint_angles = _input_joint_angles; 
}

void Forward_Kinematics::set_current_pose(std::vector<double> _current_robot_pose)
{
	current_robot_pose = _current_robot_pose;
}

std::vector<double> Forward_Kinematics::get_output_coordinates()
{
	return output_joint_coordinates;
}

std::vector<double> Forward_Kinematics::get_output_angles()
{
	return output_joint_angles;
}

std::vector<double> Forward_Kinematics::get_current_pose()
{
	return current_robot_pose;
}

std::vector<double> Forward_Kinematics::get_intput_angles()
{
   return input_joint_angles;
}

