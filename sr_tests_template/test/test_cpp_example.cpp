/*
* Copyright 2020 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
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
