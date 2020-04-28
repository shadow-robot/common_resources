# sr_utilities_common
This package contains resources commonly used in another Shadow Robot's packages and repositories

## Shutdown handler
A custom handler for SIGINT signal (ctrl + C) executing a chosen class method apon node shutdown. It's advantage over [rospy.on_shutdown](http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Registering_shutdown_hooks) is that it allows using publishers at the shutdown stage. 

Example usage:
```python
class ExampleClass():
    self.__init__():
        self.example_variable

    def example_loop_method():
        while not rospy.is_shutdown():
            rospy.loginfo("Doing some stuff!")

    def clean_up():
        rospy.loginfo("Cleaning up!")

if __name__ == '__main__':
    rospy.init_node('example_node', anonymous=True)

    example_class = ExampleClass()
    shutdown_handler = ShutdownHandler(example_class, 'clean_up()')
    example_class.example_loop_method()

```

The above code will be executing an infinite loop until shutdown for the node is initiated. Then, before the node dies, the `clean_up` method will be executed. 

## Real time tf republisher
This tool allows to republish tfs from a rosbag in real time, i.e. each tf from bag file will be intercepted and republished with current time instead of it's bagging time. Usage:

```sh
roslaunch sr_utilities_common real_time_tf_republisher.launch rosbag_path:=<rosbag_path> remapped_tf_topic_name:=<remapped_tf_topic_name>
```
where:
- `<rosbag_path>` - path to the rosbag you want to replay tf topic from (required)
- `<remapped_tf_topic_name>` - topic name to which `tf` topic from the bag will be remapped to (default: `tf_bag`)

## Ros Heartbeat
A generic ros hearbeat class has been added allowing users to enable and disable processes using a hearbeat method. In order to use it, within your node create a RosHeartbeat object:

```c++
RosHeartbeat ros_heartbeat("example_heartbeat", 0.1);
```
and use the `enabled` public variable to toggle the node. The enabled variable can be set using a publisher that is running with frequency higher then the one set in the object contructor (`10Hz` above). For example, using bash console:

```sh
rostopic pub /example_heartbeat std_msgs/Bool "data: true" -r 20
```
will set the value to `true`. The value is `true` or `false` depending on the value within the publisher's message. If the heartbeat object doesn't hear anything within it's set timeframe (`0.1c` above), user will be warned and `enabled` will be set to `false`.

## AWS Access Keys

If you wish to use the AWS CLI or our AWS Manager, you must have an AWS Access Key.

### In a new container

If you are yet to set up a container, you should do so as follows to ensure you have a customer key.

To get one, you need to install your container by running the following command: 
**N.B. This command will overwrite your current container if you have one, if you don't want that to happen, add the name tag to this command. E.g. name=my_new_container**:

```sh
bash <(curl -Ls bit.ly/run-aurora) docker_deploy --read-secure customer_key use_aws=true product=hand_e ethercat_interface=<ethercat_port> config_branch=<demohand_serial> nvidia_docker=true reinstall=true tag=melodic-release image=shadowrobot/dexterous-hand
```

Where:
- <ethercat_port> is the port to which the hand is connected, e.g. enp5s0.
- <demohand_serial> is the configuration branch of the hand, e.g. demohand_D

During installation you will be prompted for a AWS customer key. This key can be retrieved from [here](http://10.5.1.13/mediawiki/index.php/Customer_Keys_for_uploading_ROS_Logs), copy and paste one from the Table of Customer Keys.

If you can't open the link contact the software team at software@shadowrobot.com.

### In an existing container

**If you already have a container installed which does not contain an AWS key**, retrieve one of the keys from the link above and within your container run the following command:

```sh
echo "<your_aws_customer_key>" | sudo tee /usr/local/bin/customer.key
```

### AWS CLI Setup

If you wish to use the AWS CLI, you can either put your access key in the file above then run:

```sh
rosrun sr_utilities_common aws_login.py
```

Or use the access key directly:

```sh
rosrun sr_utilities_common aws_login.py --api_key <YOUR_ACCESS_KEY>
```

If you have doubts about this process, contact the software team at software@shadowrobot.com.

## AWS manager

Follow the steps above to obtain an AWS access key before attempting to use the AWS manager. It assumes you have an access key stored in `/usr/local/bin/customer.key`.

### Uploading and downloading files

An example launch file using this script could look like the one below:

```xml
<launch>
  <arg name="download" default="true"/>
  <arg name="upload" default="false"/>
  <arg name="bucket_name" default="shadowrobot.example-bucket"/>

  <arg name="files_base_path" default="$(find example_package)"/>
  <arg name="files_folder_path" default="/example_folder_within_package"/>
  <arg name="file_names" default="[example_file_0, example_file_1]"/>

  <node name="aws_manager_node" pkg="sr_utilities_common" type="aws_manager.py" output="screen">
    <rosparam param="download" subst_value="True">$(arg download)</rosparam>
    <rosparam param="upload" subst_value="True">$(arg upload)</rosparam>
    <rosparam param="files_base_path" subst_value="True">$(arg files_base_path)</rosparam>
    <rosparam param="files_folder_path" subst_value="True">$(arg files_folder_path)</rosparam>
    <rosparam param="bucket_name" subst_value="True">$(arg bucket_name)</rosparam>
    <rosparam param="file_names" subst_value="True">$(arg file_names)</rosparam>
  </node>
```

Launching the above file would upload files `example_file_0` and `example_file_1` located in package `example_package` within folder `example_folder_within_package` to the `shadowrobot.example-bucket` bucket on AWS.