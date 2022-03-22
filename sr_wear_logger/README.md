# sr_wear_logger

## sr_wear_logger class

### Description

This class is used to collect data from Shadow Hands to calculate metrics in order to estimate MTBF (Mean Time Before Failure). Currently the data being collected is the time the hand is running and the total distance traversed by each joint. 

### Diagram 

![example diagram](https://www.researchgate.net/profile/Beatriz-Bernardez/publication/279404900/figure/fig2/AS:317068311318529@1452606384897/Use-case-diagram-example.png)

### Methods 

- **check_parameters()** - verifies if the passed arguments (hand_id, serial, etc) are correct in terms of types and allowed value range.
- **run()** - starts the node by creating an instance of the AWSManager, then verifies the structure of the log file and finally starts the subscriber to /joint_states which callback methods keeps updating the current values. This methods creates 2 timers which will save the data locally and to AWS.
- **stop()** - stops the timers and kills the node

### Requirements

This class requires to have a customer key for AWS and being logged in using the aws_manager.py script.

### Usage:

```sh
roslaunch sr_wear_logger sr_wear_logger_launch.launch hand_serial:=<hand_serial> aws_save_period:=<time> local_save_period:=<time>
```
In case 'hand_serial' is not set, the node will terminate. 
The parameters 'aws_save_period' and 'local_save_period' are used to set the time (in seconds) to repeatedly save the data respectively to AWS and local.
