# sr_finger_mount

## sr_finger_mount class

This class is used to transform tactile data received from the PST's sensors into output signal for our Shadow Glove for haptic feedback.
Usage:

```sh
roslaunch sr_finger_mount sr_finger_mount_launch.launch fingers:=<comma separated finger indexes> side:=<side>
```
Example

```sh
roslaunch sr_finger_mount sr_finger_mount_launch.launch fingers:=th,ff side:=rh
```

