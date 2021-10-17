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

