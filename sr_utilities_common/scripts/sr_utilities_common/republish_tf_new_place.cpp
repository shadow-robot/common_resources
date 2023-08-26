/*
* Copyright 2021 Shadow Robot Company Ltd.
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

#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace
{
  const int RATE_FREQ = 100;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "republish_tf_new_place");

  if (argc < 5 || argc > 6)
  {
    ROS_ERROR_STREAM("Wrong number of arguments (" << argc - 1 <<
                     "), should be four or five. Terminating...");
    return 1;
  }

  std::string original_tf_parent = argv[1];
  std::string original_tf_child = argv[2];
  std::string new_tf_parent = argv[3];
  std::string new_tf_child = argv[4];

  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transform;

  ros::Rate rate = (argc == 6)
                   ? ros::Rate(std::stoi(argv[5]))
                   : ros::Rate(RATE_FREQ);

  ROS_INFO_STREAM("Republishing" << original_tf_parent << "->" << original_tf_child <<
                  " as " << new_tf_parent << "->" << new_tf_child);

  while (ros::ok())
  {
    ros::spinOnce();
    try
    {
      transform = tf_buffer.lookupTransform(original_tf_parent, original_tf_child,
                                              ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(5, "Could not get %s -> %s transform.",
        original_tf_parent.c_str(), original_tf_child.c_str());
      continue;
    }

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = new_tf_parent;
    transform.child_frame_id = new_tf_child;
    tf_broadcaster.sendTransform(transform);

    rate.sleep();
  }

  return 0;
}
