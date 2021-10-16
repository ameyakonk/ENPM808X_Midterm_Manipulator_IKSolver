#include "Inverse_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"

using std::cout;
using std::endl;

using namespace Eigen;

void set_output_coordinates(std::vector<double> _output_joint_coordinates)
{
	output_joint_coordinates = _output_joint_coordinates;
}

void set_output_angles(std::vector<double> _output_joint_angles)
{
	output_joint_angles = _output_joint_angles;
}

void set_input_angles(std::vector<double> _input_joint_angles)
{
	input_joint_angles = _input_joint_angles; 
}

void set_current_pose(std::vector<double> current_robot_pose)
{
	current_robot_pose = _current_robot_pose;
}

std::vector<double> get_output_coordinates()
{
	return output_joint_coordinates;
}

std::vector<double> get_output_angles()
{
	return output_joint_angles;
}

std::vector<double> get_current_pose()
{
	return current_robot_pose;
}

std::vector<double> get_intput_angles()
{
   return input_joint_angles;
}