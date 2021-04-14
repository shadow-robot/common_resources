# sr_wear_logger

## sr_wear_logger class

This class is used to collect data from Shadow Hands to estimate the MTBF (Mean Time Before Failure). Currently the data being collected is the time the hand is running and the total distance traversed by each joint. 
Usage:

```sh
roslaunch sr_wear_logger sr_wear_logger_launch.launch hand_serial:=<hand_serial> aws_save_period:=<time> local_save_period:=<time>
```

In case 'hand_serial' is not set, the node will terminate. 
The parameters 'aws_save_period' and 'local_save_period' are used to set the time (in seconds) to repeatedly save the data respectively to AWS and local.