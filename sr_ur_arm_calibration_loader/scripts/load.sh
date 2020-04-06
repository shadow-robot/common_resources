#!/bin/bash
(
echo "10" ; sleep 1
echo "# Launching calibration extractor" ; sleep 1
source /opt/ros/melodic/setup.bash
source /home/user/projects/shadow_robot/base/devel/setup.bash
roscore &
ros_pid=$(ps aux | grep "ros" | grep -v grep | awk '{print $2}')
echo "20" ; sleep 2
echo "# waiting" ; sleep 3
echo "50" ; sleep 1
echo "# Killing roscore" ; sleep 1
kill $ros_pid
echo "75" ; sleep 1
echo "# Finished" ; sleep 1
echo "100" ; sleep 1
) |
zenity --progress \
  --title="Calibration extration" \
  --text="Beginning calibration extraction..." \
  --percentage=0
if [ "$?" = -1 ] ; then
        zenity --error \
          --text="Update canceled."
fi
