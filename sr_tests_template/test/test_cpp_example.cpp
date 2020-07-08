/*
* Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sr_tests_template/cpp_mock_lib.h>

TEST(SrJacobianUtils, test_get_torque_given_wrench)
{
  CppMockLIb cml;
  int expected_value = cml.mock_value;
  int tested_value = 0;
  ASSERT_EQ(expected_value, tested_value);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cpp_example");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
