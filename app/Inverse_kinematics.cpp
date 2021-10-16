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