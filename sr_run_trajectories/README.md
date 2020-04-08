# sr_run_trajectories

This package contains code allowing quick and easy setup of scripts running arm, hand, and arm and hand trajectories.

## Creating trajectories

Trajectories that can be used by `SrRunTrajectories` class need to be defined in a yaml file. Each yaml file used needs to have joint order specified for each move group, as well as the actual trajectories that user desires to run. For each trajectory, multiple waypoints are allowed, and each waypoint has to have joint angles and interpolation time defined. An example, properly formatted yaml file can be found [here](./config/example_trajectories.yaml).

## Using the class

In order to use the `SrRunTrajectories` class, create an object while passing the trajectories yaml file path to the class constructor, e.g.:

```python
    trajectories_file_path = rospkg.RosPack().get_path('sr_run_trajectories') + '/config/example_trajectories.yaml'
    srt = SrRunTrajectories(trajectories_file_path)
```

In order to run a trajectory use the `run_trajectory` method. First argument of the method denotes type of the move group to be controlled (`arm`, `hand` and `arm_and_hand` allowed), second one is the name of one of the trajectories defined in the yaml file, e.g.:

```python
srt.run_trajectory('arm_and_hand', 'test_trajectory')
```

An example node executing trajectories for each move group can be found [here](./scripts/run_trajectories_node.py)

If you are running specific configuration of the robot (e.g. only arm or only hand), you can specify which robot commanders to initialize by using the class constructor,e.g.:
```python
srt = SrRunTrajectories(trajectories_file_path, arm=True, hand=False)
```

## Getting joint angles

In order to easily get joint angles in the correct order, [this script](./scripts/get_joint_angles.py) can be used. In order to run the script, run:

```sh
rosrun sr_run_trajectories get_joint_angles.py
```

 Example output:

```sh
Arm joints:
[2.352, -2.2, 0.376, -0.119, 0.448, 3.105]


Hand joints:
[0.239, 0.866, 0.417, 0.028, 0.373, 0.462, 0.874, -0.018, 0.107, 1.022, 0.749, 0.803, 0.108, 0.433, 0.583, 0.837, -0.0, 0.518, 0.171, -0.059, 1.126, -0.132, 0.034, -0.03]


Arm and hand joints:
[2.352, -2.2, 0.376, -0.119, 0.448, 3.105, 0.239, 0.866, 0.417, 0.028, 0.373, 0.462, 0.874, -0.018, 0.107, 1.022, 0.749, 0.803, 0.108, 0.433, 0.583, 0.837, -0.0, 0.518, 0.171, -0.059, 1.126, -0.132, 0.034, -0.03]
```