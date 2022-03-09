# sr_test_force_resolution

A package to move the hand E joints and generate CSV files from position, velocity and command data

## How to use

```bash
cd ~/
mkdir force_resolution_tests
cd force_resolution_tests
rosrun sr_test_force_resolution sr_test_force_resolution.py
```

Example commands: 
using effort (PWM) mode, apply a pwm of -150 to rfj3: `e_rfj3_-150`

using position mode, set all J3s to 60 degrees: `p_j3_60`

Other examples are printed when the program starts

The joint will move and the program will generate two CSV files in `~/force_resolution_tests`, one called `jointstate_{JOINTNAME}_{DATETIME}.csv` and another called `controllerstate_{JOINTNAME}_{DATETIME}.csv`

The `jointstate_...` file has the following columns:      timestamp,position,velocity,effort

The `controllerstate_...` file has the following columns: timestamp,command,setpoint,process_value,process_value_dot

To quit, type `q`
