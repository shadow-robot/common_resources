# sr_ur_arm_calibration_loader
A class that finds/generates a calibration file for whichever UR arms are currently connected.

Calling `get_calibration_files()` will also push the retrieved calibration files to the parameter server.


## How to use it

### Unimanual
```python
arm_type='UR10e'
arm1 = ('ra', '192.168.1.1', arm_type)
sr_ur_load_calibration = SrUrLoadCalibration([arm1])
arm_info = sr_ur_load_calibration.get_calibration_files()

for key, value in arm_info[0].items():
    print key, value
```

output:
```
arm_ip 192.168.1.1
arm_serial 2018301649
arm_side ra
kinematics_config /home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations/2018301649.yaml
```


### Bimanual
```python
arm_type='UR10'
arm1 = ('ra', '192.168.1.1', arm_type)
arm2 = ('la', '192.168.2.1', arm_type)
sr_ur_load_calibration = SrUrLoadCalibration([arm1, arm2])

for arm in arms_info:
    for key, value in arm.items():
        print key, value
```

output:
```
arm_ip 192.168.1.1
arm_serial 2018301649
arm_side ra
kinematics_config /home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations/2018301649.yaml
arm_ip 192.168.2.1
arm_serial 2018301236
arm_side la
kinematics_config /home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations/2018301236.yaml
```

