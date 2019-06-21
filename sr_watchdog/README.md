# Watchdog
A watchdog class that continuously runs specified check methods and informs the user about any undesired behaviours.

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