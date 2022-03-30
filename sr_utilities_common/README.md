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
bash <(curl -Ls bit.ly/run-aurora) docker_deploy --read-secure customer_key use_aws=true product=hand_e ethercat_right_hand=<ethercat_port> config_branch=<demohand_serial> nvidia_docker=true reinstall=true tag=melodic-release image=shadowrobot/dexterous-hand
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
  <arg name="function_mode" default="upload"/>
  <arg name="skip_check" default="false"/>
  <arg name="bucket_name" default="shadowrobot.example-bucket"/>
  <arg name="bucket_subfolder" default='""'/>

  <arg name="files_base_path" default="$(find example_package)"/>
  <arg name="files_folder_path" default="/example_folder_within_package"/>
  <arg name="file_names" default='""'/>

  <node name="aws_manager_node" pkg="sr_utilities_common" type="aws_manager.py" output="screen">
    <rosparam param="function_mode" subst_value="True">$(arg function_mode)</rosparam>
    <rosparam param="skip_check" subst_value="True">$(arg skip_check)</rosparam>
    <rosparam param="files_base_path" subst_value="True">$(arg files_base_path)</rosparam>
    <rosparam param="files_folder_path" subst_value="True">$(arg files_folder_path)</rosparam>
    <rosparam param="bucket_name" subst_value="True">$(arg bucket_name)</rosparam>
    <rosparam param="bucket_subfolder" subst_value="True">$(arg bucket_subfolder)</rosparam>
    <rosparam param="file_names" subst_value="True">$(arg file_names)</rosparam>
  </node>
  </launch>
```

Launching the above file would upload all files located in package `example_package` within the folder `example_folder_within_package` to the `shadowrobot.example-bucket` bucket on AWS.

You can also specify specific files by changing the file_names argument to something like `file_names:="[file1, file2]"`. You can also upload and download from specific subfolders within the bucket by adding `bucket_subfolder:="test_folder"` which would have the path `s3:shadowrobot.example-bucket/test_folder`.

## Conditional Delayed Ros Tool

This script allows to delay the launch of ros launch files and nodes. It checks on the roscore whether the required conditions have been correctly initialized until the timout exceeds.

### Fields

The node **requires** the following parameters:
- package_name: the package in which the executable is stored
- executable_name: this is the name of the launch file we want to launch (must end with .launch) or the name of the node we want to run
- timeout: after how long the script should give up waiting for the required conditions. If not set or set to zero or less, the script will wait indefinitely for the condition to be met.
- launch_args_list: the list of arguments that has to be passed to the launch file or node

The following are not mandatory and allows to define the conditions necessary to launch the wanted executable:
- topics_list: the list of topics that should be available on roscore before launching
- params_list: the list of parameters that should be available on roscore before launching
- services_list: the list of services that should be advertised before launching
- args_from_param_list: the list from arguments that will be passed down to the launch file that are read from the parameter server

You only need to specify the list you want. For instance, if there are no topics to wait for, you don't need to pass an empty list. You can also use Python's regex syntax to match specific patters, e.g. `(rh|lh)_controller` to match either `rh_controller` or `lh_controller`. Leading forward slashes in the names (`/`) are ignored, you can leave them or skip them. 

### Real code example

```xml
<node name="conditional_delayed_rostool" pkg="sr_utilities_common" type="conditional_delayed_rostool.py" output="screen">
  <param name="package_name" value="sr_teleop_launch" />
  <param name="executable_name" value="teleop_core_nodes.launch" />
  <rosparam param="topics_list">[(rh|lh)_trajectory_controller/command, ros_heartbeat]</rosparam>
  <rosparam param="params_list">[/robot_description]</rosparam>
  <rosparam param="services_list">[/get_planning_scene]</rosparam>
  <rosparam param="args_from_param_list">[robot_config_file]</rosparam>
  <param name="launch_args_list" value="sim:=$(arg sim) hand:=$(arg hand) vive:=$(arg vive) jog_arm:=$(arg jog_arm) moveit_arm:=$(arg moveit_arm) moley_arm:=$(arg moley_arm) tracker:=$(arg tracker) tracker_id:=$(arg tracker_id) wrist_wand_id:=$(arg wrist_wand_id) control_wand_id:=$(arg control_wand_id) soft_start_time:=$(arg soft_start_time) local_vive_prefix:=$(arg local_vive_prefix) user_root_tf_name:=$(arg user_root_tf_name) user_forearm_tf_name:=$(arg user_forearm_tf_name) user_wrist_tf_name:=$(arg user_wrist_tf_name) dataflow_handler_config_file_path:=$(arg dataflow_handler_config_file_path) log_topics:='$(arg log_topics)' log_bag_prefix:=$(arg log_bag_prefix) require_trigger:=$(arg require_trigger) require_pedal:=$(arg require_pedal) pedal:=$(arg pedal) side:=$(arg side) wrist_zero:=$(arg wrist_zero)" />
  <param name="timeout" value="60.0" />
</node>
```

## Killing ROS node

A `kill_node.sh` script is available to kill the node as soon as it starts. It can be used to both kill already running nodes but also to prevent nodes from starting at all and taking effect on the system. Usage:

```sh
./kill_node.sh <node_name>
```

In order to make sure that the node takes no effect on the system at all, e.g. static tfs do not appear in the tf tree, increase the priority of the process:

```sh
sudo nice -n -18 ./kill_node.sh <node_name>
```

## Ros executable wrapper

A very simple script allowing to run any non-ros executable from within a launchfile. Example usage:

```sh
    <node pkg="sr_utilities_common" type="ros_executable_wrapper.sh" name="ros_executable_wrapper_example" output="screen"
        args="non-ros-executable-script.sh exemple_arg_1 exemple_arg_2/>

```

## Republishing tfs from a new place

A ros node allowing to take existing values of a tf and republish it from a difference parent frame with desired child frame name. Usage:

```sh
rosrun sr_utilities_common republish_tf_new_place <original_parent_tf> <original_child_tf> <new_parent_tf> <new_child_tf>
```

If you want the tf to be published with specific frequency (default is 100 Hz) you can add another argument to the end of the command:

```sh
rosrun sr_utilities_common republish_tf_new_place <original_parent_tf> <original_child_tf> <new_parent_tf> <new_child_tf> <publishing_frequency>
```
