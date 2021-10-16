#include "Inverse_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"

using std::cout;
using std::endl;

void Inverse_Kinematics::set_input_coordinates(std::vector<double> _input_joint_coordinates) {
  input_joint_coordinates = _input_joint_coordinates;
}

std::vector<double> Inverse_Kinematics::get_input_coordinates() {
 return input_joint_coordinates;
}

void Inverse_Kinematics::set_input_angles(std::vector<double> _input_joint_angles) {
  input_joint_angles = _input_joint_angles;
}

std::vector<double> Inverse_Kinematics::get_input_angles() {
 return input_joint_angles;
}

void Inverse_Kinematics::set_output_angles(std::vector<double> _output_joint_angles) {
  output_joint_angles = _output_joint_angles;
}

std::vector<double> Inverse_Kinematics::get_output_angles() {
 return output_joint_angles;
}

void Inverse_Kinematics::set_output_coordinates(std::vector<double> _output_joint_coordinates) {
  output_joint_coordinates = _output_joint_coordinates;
}

std::vector<double> Inverse_Kinematics::get_output_coordinates() {
 return output_joint_coordinates;
}

void Inverse_Kinematics::set_current_pose(std::vector<double> _current_robot_pose) {
  current_robot_pose = _current_robot_pose;
}

std::vector<double> Inverse_Kinematics::get_current_pose() {
 return current_robot_pose;
}

void Inverse_Kinematics::set_dh_a(std::vector<double> _dh_a) {
   dh_a = _dh_a;
}

std::vector<double> Inverse_Kinematics::get_dh_a() {
 return dh_a;
}

void Inverse_Kinematics::set_dh_d(std::vector<double> _dh_d) {
   dh_d = _dh_d;
}

std::vector<double> Inverse_Kinematics::get_dh_d() {
 return dh_d;
}

void Inverse_Kinematics::set_dh_alpha(std::vector<double> _dh_alpha) {
   dh_alpha = _dh_alpha;
}

std::vector<double> Inverse_Kinematics::get_dh_alpha() {
 return dh_alpha;
}

void Inverse_Kinematics::reset_pose() {

}
