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
#include <ros/package.h>
#include "sr_hand_detector/sr_hand_detector.h"
#include "sr_hand_detector/sr_hand_autodetect.h"


class MockSrHandDetectorUnimanual : public sr_hand_detector::SrHandDetector
{
 public:
  MockSrHandDetectorUnimanual()
  {
    hand_serial_and_port_map_.insert(std::pair<int, std::string>(1130, "eth0"));
  }
};

class MockSrHandDetectorBimanual : public sr_hand_detector::SrHandDetector
{
 public:
  MockSrHandDetectorBimanual()
  {
    hand_serial_and_port_map_.insert(std::pair<int, std::string>(1130, "eth0"));
    hand_serial_and_port_map_.insert(std::pair<int, std::string>(2346, "eth1"));
  }
};

TEST(SrHandAutodetect, test_run_unimanual)
{
  std::string expected_command_sufix = " eth_port:=eth0 hand_serial:=1130 hand_id:=rh";
  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";

  MockSrHandDetectorUnimanual mock_sr_hand_detector;
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(mock_sr_hand_detector, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.command_sufix_, expected_command_sufix);
}

TEST(SrHandAutodetect, test_run_bimanual)
{
  std::string expected_command_sufix = " eth_port:=eth0_eth1 rh_serial:=1130 lh_serial:=2346";
  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";


  MockSrHandDetectorBimanual mock_sr_hand_detector;
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(mock_sr_hand_detector, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.command_sufix_, expected_command_sufix);
}

TEST(SrHandAutodetect, test_run_no_hands)
{
  std::string expected_command_sufix = "";

  sr_hand_detector::SrHandDetector sr_hand_detector;
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(sr_hand_detector);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.command_sufix_, expected_command_sufix);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sr_hand_autodetect");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}