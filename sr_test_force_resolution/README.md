# sr_test_force_resolution

A package to move the hand E joints and generate CSV files from position, velocity and command data

## Prerequisites 

```bash
pip install qprompt
```


## How to use

```bash
cd ~/
mkdir force_resolution_tests
cd force_resolution_tests
rosrun sr_test_force_resolution sr_test_force_resolution.
```

A text menu will show with further instructions

The joint will move and the program will generate two CSV files in `~/force_resolution_tests`, one called `jointstate_{JOINTNAME}_{DATETIME}.csv` and another called `controllerstate_{JOINTNAME}_{DATETIME}.csv`

The `jointstate_...` file has the following columns:      timestamp,position,velocity,effort

The `controllerstate_...` file has the following columns: timestamp,command,setpoint,process_value,process_value_dot

To quit, type `q`


## ToDo

Remove dependency on qprompt

Get joint direction signs automatically

Get min/max PWM automatically
