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