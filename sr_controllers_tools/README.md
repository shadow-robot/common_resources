## SR_CONTROLLER_TOOLS

Package that contains modules to switch, load and run hand E ROS controllers.

### [sr_controller_helper](https://github.com/shadow-robot/common_resources/blob/melodic-devel/sr_controllers_tools/src/sr_controllers_tools/sr_controller_helper.py)

This module contains API functions to switch controllers for hand E.


In order to use this module import it in your python script as shown in the line below:

```
from sr_controllers_tools.sr_controller_helper import ControllerHelper
```

Then ControllerHelper object needs to be initialised by passing:

- a list of hand_prefixes (e.g. rh or lh)
- a list of robot_joint_prefixes (e.g. rh_ or lh_)
- the list of robot joint names


#### Changing hand control type

The hand control type can be changed from "position" to "effort" and viceversa. 
An example call to switch the hand to PWM mode, would look like in the code snippet below:
```
control_type = "effort"
self.controller_helper.change_hand_ctrl(control_type)
```

#### Hand trajectory controller

The trajectory controller for the hand can also be changed from "run" to "stop" and viceversa.
An example call is shown in the code snippet below:

```
self.controller_helper.change_trajectory_ctrl("run")
```
