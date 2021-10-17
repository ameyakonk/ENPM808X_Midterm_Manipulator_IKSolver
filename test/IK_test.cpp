#include <gtest/gtest.h>
#include "Inverse_kinematics.hpp"
#define PI 3.14

TEST(get_input_coordinates_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i;
  I.set_input_coordinates({20, 20, 20});
  std::vector <double> _input_coordinates({20, 20, 20});
  for(i=0 ; i<3; i++)ASSERT_EQ(_input_coordinates[i], I.get_input_coordinates()[i]);
  //ASSERT_THROW(I.get_input_coordinates(), ElementsAre(20, 20, 20));
}

TEST(get_input_angles_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_input_angles({PI/2, PI/4, PI/6});
  std::vector <double> _input_angles({PI/2, PI/4, PI/6});
  for(i=0 ; i<2; i++)ASSERT_EQ(_input_angles[i], I.get_input_angles()[i]);
}

TEST(get_output_angles_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_output_angles({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _output_angles({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for(i=0 ; i<6; i++)ASSERT_EQ(_output_angles[i], I.get_output_angles()[i]);
}

TEST(get_current_pose_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_current_pose({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _current_pose({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for(i=0 ; i<6; i++)ASSERT_EQ(_current_pose[i], I.get_current_pose()[i]);
}

TEST(get_dh_a_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_dh_a({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _dh_a({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for(i=0 ; i<6; i++)ASSERT_EQ(_dh_a[i], I.get_dh_a()[i]);
}

TEST(get_dh_d_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_dh_d({0, 5, 10, 0, 0, 0});
  std::vector <double> _dh_d({0, 5, 10, 0, 0, 0});
  for(i=0 ; i<6; i++)ASSERT_EQ(_dh_d[i], I.get_dh_d()[i]);
}

TEST(get_dh_alpha_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.set_dh_alpha({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  std::vector <double> _dh_alpha({PI/2, PI/4, PI/6, PI/2, PI/4, PI/6});
  for(i=0 ; i<6; i++)ASSERT_EQ(_dh_alpha[i], I.get_dh_alpha()[i]);
}

TEST(get_convert_input_angles_to_rotation_matrix_IK, should_return_set_values) {
  
  Inverse_Kinematics I;
  std::vector <double>::size_type i=0;
  I.convert_input_angles_to_rotation_matrix({PI/2, PI/4, PI/4});
  std::vector <double> _dh_alpha({-1.57, 1.57, 0, -1.57, 1.57, 0});
  for(i=0 ; i<6; i++)ASSERT_EQ(_dh_alpha[i], I.get_dh_alpha()[i]);
}
