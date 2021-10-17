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

TEST(get_output_angles_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i = 0;
  F.set_output_angles( { PI / 2, PI / 4, PI / 6 });
  std::vector<double> _output_angles( { PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 2; i++)
    ASSERT_EQ(_output_angles[i], F.get_output_angles()[i]);
}

TEST(get_current_pose_FK, should_return_set_values) {

  Forward_Kinematics F;
  std::vector<double>::size_type i = 0;
  F.set_current_pose( { PI / 2, PI / 4, PI / 6, PI / 2, PI / 4, PI / 6 });
  std::vector<double> _current_pose(
      { PI / 2, PI / 4, PI / 6, PI / 2, PI / 4, PI / 6 });
  for (i = 0; i < 6; i++)
    ASSERT_EQ(_current_pose[i], F.get_current_pose()[i]);
}
