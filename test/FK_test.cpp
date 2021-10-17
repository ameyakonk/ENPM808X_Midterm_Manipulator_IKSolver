#include <gtest/gtest.h>
#include "Forward_kinematics.hpp"
#define PI 3.14

TEST(get_output_coordinates_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i;
  F.set_output_coordinates( { 20, 20, 20 });
  std::vector<double> _output_coordinates( { 20, 20, 20 });
  for (i = 0; i < 3; i++)
    ASSERT_EQ(_output_coordinates[i], F.get_output_coordinates()[i]);
}
