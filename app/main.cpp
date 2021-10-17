#include <iostream>
#include <Eigen/Core>
#include "Inverse_kinematics.hpp"
#include "Forward_kinematics.hpp"
#define PI 3.14
using namespace Eigen;
using std::cout;
using std::endl;
int main()
{
	Inverse_Kinematics I;
	Forward_Kinematics F;
	std::vector<double> temp_input_joint_angles {PI/2, PI/4, PI/4};
	I.convert_input_angles_to_rotation_matrix(temp_input_joint_angles);
	//F.solve_FK(I.get_output_angles());
	std::vector <double>::size_type i=0;

}
