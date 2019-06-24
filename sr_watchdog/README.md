# Watchdog
A watchdog class that continuously runs specified check methods and informs the user about any undesired behaviours.

<p align="center">
  <img src="./doc/watchdog_gui.png" alt="demo setup"/>
</p>

An example of how to properly use the class can be seen in the code snipper below:
```python
#!/usr/bin/python

import rospy
from sr_utilities_common.shutdown_handler import ShutdownHandler
from sr_utilities_common.watchdog import SrWatchdog

class TestChecksClass(object):
    def __init__(self):
        self.tmp = [0, 0]

    def mock_check_robot_clear_from_collision(self):
        rospy.sleep(4)
        if self.tmp[0] != 1:
            self.tmp[0] = 1
            return False
        else:
            self.tmp[0] = 0
            return True

    def mock_check_if_arm_running(self):
        rospy.sleep(5)
        if self.tmp[1] != 0:
            self.tmp[1] = 0
            return (False, "Something went wrong!")
        else:
            self.tmp[1] = 1
            return True


if __name__ == '__main__':
    rospy.init_node('mock_sr_teleop_watchdog')

    test_class = TestChecksClass()
    error_checks_list = ['mock_check_if_arm_running']
    warning_checks_list = ['mock_check_robot_clear_from_collision']

    teleop_watchdog = SrWatchdog(test_class, error_checks_list, warning_checks_list)
    teleop_watchdog.run()
    rospy.spin()
```

All the check methods are defined in a separate class. There are two types of checks supported. Error checks, which will change general status to `error` and throw an error message, and warning checks, which do not affect the reported status but inform the user that something has happened. In order to classify a check as error or warning check, their method name needs to be put in a proper list that is further passed to the `SrWatchdog` class constructor, together with the checks class object.

If all checks pass, the watchdog reports the `OK` status for the demo. This status is maintained every iteration of the checking cycle if the checks keep passing. In any iteration, if one of the error checks fails, the status of the demo will be changed to `Error` and the led light will turn red. If a warning check fails, the `OK` status will be maintained, however, led will turn yellow. All failing checks will be displayed in the red textbox below the status label. Finally, if any of the checks starts passing again, it will be removed from the box and if all error checks start passing again, demo status will go back to `OK`.

The check methods are required to return either `True` (for passed check) or `False` (for failed check) values. Alternatively, in case of failure, error checks can return a tuple of `(False, <error message>)` where `<error message>` is additional, user-defined error message.

By default, the watchdog class publishes result to the `sr_watchdog` topic. However, a GUI is available to visualize the system status and logs. In order to run the GUI, after starting your watchdog, run

```sh
roslaunch sr_watchdog watchdog_web_gui.launch
```

and in the browser of your choice go to

```sh
localhost:8080
```

An example, mock watchdog has been implemented [here](./scripts/mock_watchdog.py) and can be run, together with the GUI using [this launch file](./launch/mock_watchdog.launch).