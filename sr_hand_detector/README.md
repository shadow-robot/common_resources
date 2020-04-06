# sr_hand_detector

This package allows user to detect Shadow Hands without knowing the ethernet interface or the hand serial. Usage:

```sh
roslaunch sr_hand_detector sr_hand_detector.launch
```

Example output:
```sh
[ INFO] [1585576030.675150873]: Detected hand on port: enx000ec6bfe185
[ INFO] [1585576030.675215678]: Hand's serial number: 1130
```

Apart from the console output, all detected hand ethernet port names together with corresponding hand serial numbers will be set inside of the `sr_hand_detector` parameter on the parameter server.

If there are no hands detected on any of the ports, a warning will be shown:
```sh
[ WARN] [1585576995.520983706]: No hand detected on any of the ports!
```
and the node will terminate with exit code `1`.

Keep in mind that if not using the provided launchfile, you will have to run the node using ethercat_grant or with sudo privileges.