#include "Inverse_kinematics.hpp"
#include <cstdlib>
#include <cmath>
#include "Eigen/Core"

using std::cout;
using std::endl;

void Inverse_Kinematics::convert_input_angles_to_rotation_matrix(std::vector<double> input_joint_angles) {

  std::vector <double>::size_type i=1;
  Eigen::Matrix <double, 3, 3> ROLL;
  ROLL <<   1.0f, 0.0f, 0.0f,
            0.0f, cos(input_joint_angles[i-1]), (-sin(input_joint_angles[i-1])),
            0.0f, sin(input_joint_angles[i-1]), cos(input_joint_angles[i-1]);

  Eigen::Matrix <double, 3, 3> PITCH;
  PITCH <<   cos(input_joint_angles[i]),    0.0f, sin(input_joint_angles[i]),
             0.0f,                            1.0f,              0.0f,
             (-sin(input_joint_angles[i])), 0.0f, cos(input_joint_angles[i]);

  Eigen::Matrix <double, 3, 3> YAW;
  YAW <<   cos(input_joint_angles[i+1]), (-sin(input_joint_angles[i+1])), 0.0f,
           sin(input_joint_angles[i+1]), cos(input_joint_angles[i+1]),    0.0f,
           0.0f,                          0.0f,                           1.0f;

  Eigen::Matrix <double, 3, 3> ROTATION_MATRIX;
  ROTATION_MATRIX = YAW*PITCH*ROLL;

  std::vector<double> rotation_matrix;
  for(int r=0 ; r < 9; r++) rotation_matrix.push_back(ROTATION_MATRIX(r)); 
  set_input_angles(rotation_matrix);
  set_input_coordinates({20, 20, 20});
  set_dh_d({0, 5, 10, 0, 0, 0});
  set_dh_a({0, 0, 0, 0, 0, 0});
  set_dh_alpha({-PI/2, PI/2, 0, (-PI/2), PI/2, 0});
  solve_IK(get_input_coordinates(), get_input_angles());
}


void Inverse_Kinematics::solve_IK(std::vector<double> input_joint_coordinates, std::vector<double> input_joint_angles) {

  double* theta = (double *)malloc(sizeof(double)*7);
        
  std::vector <double>::size_type i=1;
  
  double r = pow(pow(input_joint_coordinates[i-1],2) + pow(input_joint_coordinates[i],2), 0.5);
  
  theta[0] = atan(input_joint_coordinates[i]/input_joint_coordinates[i-1]) - atan(dh_d[i]/pow(pow(r, 2) - pow(dh_d[i],2) , 0.5));

  theta[1] = atan(((cos(theta[0])*input_joint_coordinates[i-1]) - (sin(theta[0])*input_joint_coordinates[i]))/input_joint_coordinates[i+1]);
  
  double d3 = (input_joint_coordinates[i-1]*cos(theta[0]) + sin(theta[0])*input_joint_coordinates[i])*sin(theta[1]) + input_joint_coordinates[i+1]*cos(theta[1]);

  theta[3] = atan(((-sin(theta[0]))*input_joint_angles[i+5] + cos(theta[0])*input_joint_angles[i+6])/
            (cos(theta[1])*(cos(theta[0])*input_joint_angles[i+5] + sin(theta[0])*input_joint_angles[i+6])- sin(theta[1])*input_joint_angles[i+6]));

  double I = cos(theta[0])*input_joint_angles[i+3] + sin(theta[0])*input_joint_angles[i+4];

  double n = (-sin(theta[0]))*input_joint_angles[i+2] + cos(theta[0])*input_joint_angles[i+3]; 

  theta[4] = atan((cos(theta[3])*(cos(theta[1])*(cos(theta[0])*input_joint_angles[i+5] + sin(theta[0])*input_joint_angles[i+5]) - sin(theta[1])*input_joint_angles[i+7]) + sin(theta[3])*(cos(theta[0])*input_joint_angles[i+6] - sin(theta[0])*input_joint_angles[i+5]))/
                  (sin(theta[1])*(cos(theta[0])*input_joint_angles[i+5] + sin(theta[0])*input_joint_angles[i+6])+ cos(theta[1])*input_joint_angles[i+7]));

  theta[5] = atan(((-cos(theta[4]))*(cos(theta[3])*(cos(theta[1])*I - sin(theta[1])*input_joint_angles[i+4]) + sin(theta[3])*n) + sin(theta[4])*(sin(theta[1])*I + cos(theta[1])*input_joint_angles[i+4]))/
                      (-sin(theta[3])*(cos(theta[1])*I - sin(theta[1])*input_joint_angles[i+4])+ cos(theta[3])*n));

  std::vector<double> output_joint_angles;
  std::vector<double> _dh_d {0, 5, d3, 0, 0, 0};
  
  set_dh_d(_dh_d);

  for(int i=0; i<6; i++)output_joint_angles.push_back(theta[i]);
  
  free(theta);
  
  set_output_angles(output_joint_angles); 
}

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
