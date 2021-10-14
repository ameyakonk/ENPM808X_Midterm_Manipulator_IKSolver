#include <iostream>
#include <lib.hpp>
#include <Manipulator.hpp>
#include <kinematics.hpp>
#define PI 3.14

using std::cout;
using std::endl;

int main()
{
    Kinematics k;
    std::vector<double> temp_joint_coordinates {1.0, 1.0, 1.0};
    std::vector<double> temp_joint_angles {PI/2, PI/4, PI/4, PI/4, PI/4, PI/4, PI/4, PI/4, PI/4};
    k.set_input_coordinates(temp_joint_coordinates);
    k.set_input_angles(temp_joint_angles);
    std::vector<double> print = k.solve_IK(k.get_input_coordinates(), k.get_input_angles()); 
    std::vector<double>::size_type i=0;
    for(i=0; i<6; i++) {
    	cout << k.solve_IK(k.get_input_coordinates(), k.get_input_angles())[i] << endl;
    }
    i=0;
    double s = pow(pow((k.get_input_coordinates()[i+1]) - k.get_dh_d()[i],2), 0.5);
    cout << s;

    return 0;
}
